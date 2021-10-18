#include <kuam_aruco_tracking/aruco_tracking.h>

#include <geometry_msgs/Point.h>

#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <kuam_msgs/ArucoVisual.h>
#include <kuam_msgs/ArucoVisuals.h>
#include <kuam_msgs/marker_ids.h>
#include <algorithm>

#include <kuam_msgs/FittingPlane.h>

using namespace std;
using namespace cv;

namespace kuam{
    
ArucoTracking::ArucoTracking() :
    m_p_nh("~"),
    MARKER_ID_QUEUE_SIZE(30)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[aruco_tracking] Fail GetParam %s", m_err_param.c_str());
    InitROS();
    InitMarker();

    //namedWindow(OPENCV_WINDOW);
    m_parser.ReadFile(m_dictionaryID_param, m_calib_path_param, m_detector_params_path_param,
                    m_detector_params, m_dictionary, m_cam_matrix, m_dist_coeffs);
}

ArucoTracking::~ArucoTracking()
{
    //destroyWindow(OPENCV_WINDOW);
}

bool ArucoTracking::InitFlag()
{
    m_fix_small_marker = false;
    m_z_to_big.is_init = false;
    m_z_to_medium.is_init = false;
    m_z_to_small.is_init = false;
    return true;
}

bool ArucoTracking::GetParam()
{
    string ns_name = ros::this_node::getNamespace();

    if (!m_p_nh.getParam("calib_path", m_calib_path_param))  { m_err_param = "calib_path"; return false; }
    if (!m_p_nh.getParam("detector_params_path", m_detector_params_path_param))  { m_err_param = "detector_params_path"; return false; }
    if (!m_p_nh.getParam("camera_frame_id", m_camera_frame_id_param))  { m_err_param = "camera_frame_id"; return false; }
    if (!m_p_nh.getParam("compare_mode", m_compare_mode_param))  { m_err_param = "compare_mode"; return false; }
    if (!m_p_nh.getParam("simulation", m_is_simulation_param))  { m_err_param = "simulation"; return false; }
    if (!m_p_nh.getParam("dictionaryID", m_dictionaryID_param))  { m_err_param = "dictionaryID"; return false; }
    if (!m_p_nh.getParam("filter_buf_size", m_filter_buf_size_param))  { m_err_param = "filter_buf_size"; return false; }
    if (!m_p_nh.getParam("estimating_method", m_estimating_method_param))  { m_err_param = "estimating_method"; return false; }
    if (!m_p_nh.getParam("noise_dist_th_m", m_noise_dist_th_m_param))  { m_err_param = "noise_dist_th_m"; return false; }
    if (!m_p_nh.getParam("noise_cnt_th", m_noise_cnt_th_param))  { m_err_param = "noise_cnt_th"; return false; }
    if (!m_p_nh.getParam("process_freq", m_process_freq_param))  { m_err_param = "process_freq"; return false; }
    if (!m_p_nh.getParam("marker_cnt_th", m_marker_cnt_th_param))  { m_err_param = "marker_cnt_th"; return false; }
    if (!m_p_nh.getParam("plane_threshold", m_plane_threshold_param))  { m_err_param = "plane_threshold"; return false; }
    if (!m_p_nh.getParam("plane_iterations", m_plane_iterations_param))  { m_err_param = "plane_iterations"; return false; }
    if (!m_p_nh.getParam("angle_deg_threshold", m_angle_deg_threshold_param))  { m_err_param = "angle_deg_threshold"; return false; }

    if (!m_p_nh.getParam("marker_size_type_num", m_marker_size_type_num_param))  { m_err_param = "marker_size_type_num"; return false; }
    if (!m_p_nh.getParam("big_marker_trans", m_big_marker_trans_param))  { m_err_param = "big_marker_trans"; return false; }
    if (!m_p_nh.getParam("medium_marker_trans", m_medium_marker_trans_param))  { m_err_param = "medium_marker_trans"; return false; }
    if (!m_p_nh.getParam("small_marker_trans", m_small_marker_trans_param))  { m_err_param = "small_marker_trans"; return false; }


    for (int i = 0; i < m_marker_size_type_num_param; i++){
        string param_name = "marker_ids_type_" + to_string(i);

        vector<int> marker_ids;
        XmlRpc::XmlRpcValue list;
        if (!m_p_nh.getParam(param_name, list))  { m_err_param = param_name; return false; }
        ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < list.size(); ++i) {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
            int id = static_cast<int>(list[i]);
            marker_ids.push_back(id);
        }
        m_marker_ids_vec.push_back(marker_ids);

        
        if (!m_p_nh.getParam("marker_sizes_m", list))  { m_err_param = "marker_sizes_m"; return false; }
        ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < list.size(); ++i) {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
            double size = static_cast<double>(list[i]);
            m_marker_sizes_m.push_back(size);
        }
    }

    return true;
}

bool ArucoTracking::InitROS()
{
    // package, node, topic name
    string image_topic_name;
    if (m_is_simulation_param) image_topic_name = "/camera/rgb/image_raw";
    else image_topic_name = "usb_cam/image_rect_color";

    // Initialize subscriber
    m_image_sub = m_nh.subscribe<sensor_msgs::Image>(image_topic_name, 1, boost::bind(&ArucoTracking::ImageCallback, this, _1));
    
    // Initialize publisher
    m_image_pub = m_p_nh.advertise<sensor_msgs::Image>("image_raw", 10);
    m_tf_list_pub = m_p_nh.advertise<tf2_msgs::TFMessage>("tf_list", 10);
    m_visual_pub = m_p_nh.advertise<kuam_msgs::ArucoVisuals> ("aruco_visuals", 1);
    m_target_state_pub = m_p_nh.advertise<kuam_msgs::ArucoStates> ("target_states", 1);
    m_fitting_plane_pub = m_p_nh.advertise<kuam_msgs::FittingPlane> ("fitting_plane", 1);
    
    // Initialize timer
    m_image_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &ArucoTracking::ProcessTimerCallback, this);

    return true;
}

bool ArucoTracking::InitMarker()
{
    int idx = 0;
    for (auto ids : m_marker_ids_vec){
        for (auto id : ids){
            Target target(id, m_marker_sizes_m[idx], m_filter_buf_size_param, m_noise_cnt_th_param, 
                m_noise_dist_th_m_param, m_estimating_method_param, m_compare_mode_param);
            
            m_targets.push_back(target);
            m_marker_ids.push_back(id);
        }
        idx++;
    }
}



void ArucoTracking::ProcessTimerCallback(const ros::TimerEvent& event)
{
    if (m_cv_ptr == nullptr){
        return;
    }
    // Marker detection by using opencv
    Mat image;
    image = m_cv_ptr->image;
    image = image(Rect(40, 40, 600, 440));
    vector<int> ids;
    vector<vector<Point2f>> corners;
    vector<vector<Point2f>> rejected;
    aruco::detectMarkers(image, m_dictionary, corners, ids, m_detector_params, rejected);


    // Id distribution
    vector < vector<int> > discrete_ids_vec;
    vector < vector<vector<Point2f>> > discrete_corners_vec;
    for (auto target_ids : m_marker_ids_vec){
        vector<int> discrete_ids;
        vector<vector<Point2f>> discrete_corners;
        SelectMarker(target_ids, corners, ids, discrete_corners, discrete_ids);

        discrete_ids_vec.push_back(discrete_ids);
        discrete_corners_vec.push_back(discrete_corners);
    }

    // Noise filter
    NoiseFilter(discrete_corners_vec, discrete_ids_vec);

    bool is_empty = true;
    for (auto ids : discrete_ids_vec){
        if (!ids.empty()) is_empty = false;
    }
    if (is_empty){
        return;
    }

    // Estimate marker pose by using opencv
    vector < vector<Vec3d> > rvecs_vec, tvecs_vec;
    for (auto i = 0; i < discrete_corners_vec.size(); i++){
        vector<Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(discrete_corners_vec[i], m_marker_sizes_m[i], 
                                        m_cam_matrix, m_dist_coeffs, rvecs, tvecs);

        rvecs_vec.push_back(rvecs);
        tvecs_vec.push_back(tvecs);
    }

    // Get transformation from camera to marker
    int2pose int_to_pose;
    tf2_msgs::TFMessage tf_msg_list;
    for (auto i = 0; i < discrete_corners_vec.size(); i++){
        GetTransformation(discrete_corners_vec[i], discrete_ids_vec[i], rvecs_vec[i], tvecs_vec[i], int_to_pose, tf_msg_list);

    }

    // Update marker is_detection and pose (one of three method, no filter, average fitler, exponential average filter)
    MarkerUpdate(discrete_ids_vec, int_to_pose);

    // Get marker messages
    kuam_msgs::ArucoStates ac_states_msg;
    kuam_msgs::ArucoVisuals ac_visuals_msg;
    SetArucoMessages(ac_states_msg, ac_visuals_msg);

    // Publish
    m_tf_list_pub.publish(tf_msg_list);
    ImagePub(image, discrete_corners_vec, discrete_ids_vec);
    TargetPub(ac_states_msg, ac_visuals_msg);
}

void ArucoTracking::ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr)
{
    if(!m_util_marker.Convert2CVImg(img_ptr, m_cv_ptr)) ROS_ERROR("[aruco_tracking] Fail to convert sensor_msgs to cv_image");
}

void ArucoTracking::SelectMarker(const vector<int> target_ids, vector<vector<Point2f>>& corners, vector<int>& ids,
    vector<vector<Point2f>>& discrete_corners, vector<int>& discrete_ids)

{
    if (ids.empty()){
        return;
    }

    m_util_marker.GetIdsnCorners(target_ids, ids, corners, discrete_ids, discrete_corners);
    m_util_marker.EraseIdnCorner(discrete_ids, ids, corners);
}

void ArucoTracking::NoiseFilter(vector < vector<vector<Point2f>> >& corners_vec, vector < vector<int> >& ids_vec)
{
    auto idx = 0;

    for (auto i = 0; i < corners_vec.size(); i++){
        vector<int> noises;
        m_util_marker.GetNoiseIndexes(noises, ids_vec[i], m_targets);
        m_util_marker.EraseIdnCorner(noises, ids_vec[i], corners_vec[i]);
    }
}

void ArucoTracking::MarkerUpdate(const vector < vector<int> > ids_vec, int2pose int_to_pose)
{
    vector<int> detected_ids;
    vector<int> undetected_ids;
    for (auto i = 0; i < ids_vec.size(); i++){
        m_util_marker.GetDetectedId(detected_ids, undetected_ids, ids_vec[i], m_marker_ids_vec[i]);
    }

    for (auto id : detected_ids){
        for (auto& target : m_targets){
            if (target.GetId() != id){
                continue;
            }

            target.UpdateDetection(true);

            auto it = int_to_pose.find(id);
            if (it != int_to_pose.end()){
                auto p_x = it->second.position.x;
                auto p_y = it->second.position.y;
                auto p_z = it->second.position.z;
                auto q_x = it->second.orientation.x;
                auto q_y = it->second.orientation.y;
                auto q_z = it->second.orientation.z;
                auto q_w = it->second.orientation.w;
                
                target.TargetStateEstimating(p_x, p_y, p_z, q_x, q_y, q_z, q_w);
            }
            else{
                target.TargetStateEstimating();
            }
            break;
        }
    }

    for (auto id : undetected_ids){
        for (auto& target : m_targets){
            if (target.GetId() != id){
                continue;
            }

            target.UpdateDetection(false);
            target.TargetStateEstimating();
            break;
        }
    }
}

void ArucoTracking::SetArucoMessages(kuam_msgs::ArucoStates& ac_states_msg, kuam_msgs::ArucoVisuals& ac_visuals_msg)
{
    ac_states_msg.header.frame_id = m_camera_frame_id_param;
    ac_states_msg.header.stamp = ros::Time::now();
    
    // Insert each detected aruco marker
    for (auto target : m_targets){
        kuam_msgs::ArucoState ac_state_msg;
        ac_state_msg.header.frame_id = m_camera_frame_id_param;
        ac_state_msg.header.stamp = ros::Time::now();
        ac_state_msg.id = target.GetId();

        if (target.GetIsDetected()){
            ac_state_msg.is_detected = true;
            
            ac_state_msg.pose.position.x = target.GetX(m_estimating_method_param);
            ac_state_msg.pose.position.y = target.GetY(m_estimating_method_param);
            ac_state_msg.pose.position.z = target.GetZ(m_estimating_method_param);
            
            ac_state_msg.pose.orientation.x = target.GetQX();
            ac_state_msg.pose.orientation.y = target.GetQY();
            ac_state_msg.pose.orientation.z = target.GetQZ();
            ac_state_msg.pose.orientation.w = target.GetQW();
        }
        else{
            ac_state_msg.is_detected = false;
        }
        ac_states_msg.aruco_states.push_back(ac_state_msg);

        kuam_msgs::ArucoVisual aruco_visual_msg;
        aruco_visual_msg.header.frame_id = m_camera_frame_id_param;
        aruco_visual_msg.header.stamp = ros::Time::now();
        aruco_visual_msg.is_compare_mode = m_compare_mode_param;
        aruco_visual_msg.id = target.GetId();
        aruco_visual_msg.esti_method =  m_estimating_method_param;

        if (target.GetIsDetected()){
            aruco_visual_msg.is_detected = true;
            for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
                geometry_msgs::Pose pose;
                pose.position.x = target.GetX(method);
                pose.position.y = target.GetY(method);
                pose.position.z = target.GetZ(method);
                aruco_visual_msg.poses.push_back(pose);
            }
        }
        else{
            aruco_visual_msg.is_detected = false;
        }
        ac_visuals_msg.aruco_visuals.push_back(aruco_visual_msg);
    }

    // // Set target pose
    // std::shared_ptr<vector<kuam_msgs::ArucoState>> detected_ac_states = make_shared<vector<kuam_msgs::ArucoState>>();
    // kuam_msgs::FittingPlane fitting_plane_msg;
    
    // vector<kuam_msgs::ArucoState> markers = m_util_marker.GetMarkerState(m_big_marker_ids_param, ac_states_msg);
    // geometry_msgs::PoseStamped big_plane;
    // bool is_valid = false;
    // if (m_util_setpoint.GeneratePlane(markers, big_plane, m_z_to_big, m_plane_threshold_param, 
    //                                 m_plane_iterations_param, m_angle_deg_threshold_param, is_valid)){
    //     if (is_valid){
    //         detected_ac_states->insert(detected_ac_states->end(), markers.begin(), markers.end());
    //         ac_states_msg.is_detected = true;
    //     }
    //     fitting_plane_msg.big_is_valid = true;
    //     fitting_plane_msg.big_plane = big_plane;
    //     fitting_plane_msg.big_z_to_normal_deg = m_z_to_big.angle_deg;
    // }
    
    // markers.clear();
    // markers = m_util_marker.GetMarkerState(m_medium_marker_ids_param, ac_states_msg);
    // geometry_msgs::PoseStamped medium_plane;
    // is_valid = false;
    // if (m_util_setpoint.GeneratePlane(markers, medium_plane, m_z_to_medium, m_plane_threshold_param, 
    //                                 m_plane_iterations_param, m_angle_deg_threshold_param, is_valid)){
    //     if (is_valid){
    //         detected_ac_states->insert(detected_ac_states->end(), markers.begin(), markers.end());
    //         ac_states_msg.is_detected = true;
    //     }
    //     fitting_plane_msg.medium_is_valid = true;
    //     fitting_plane_msg.medium_plane = medium_plane;
    //     fitting_plane_msg.medium_z_to_normal_deg = m_z_to_medium.angle_deg;
    // }

    // markers.clear();
    // markers = m_util_marker.GetMarkerState(m_small_marker_ids_param, ac_states_msg);
    // geometry_msgs::PoseStamped small_plane;
    // is_valid = false;
    // if (m_util_setpoint.GeneratePlane(markers, small_plane, m_z_to_small, m_plane_threshold_param, 
    //                                 m_plane_iterations_param, m_angle_deg_threshold_param, is_valid)){
    //     if (is_valid){
    //         detected_ac_states->insert(detected_ac_states->end(), markers.begin(), markers.end());
    //         ac_states_msg.is_detected = true;
    //     }
    //     fitting_plane_msg.small_is_valid = true;
    //     fitting_plane_msg.small_plane = small_plane;
    //     fitting_plane_msg.small_z_to_normal_deg = m_z_to_small.angle_deg;
    // }

    // if (ac_states_msg.is_detected){
    //     vector<geometry_msgs::Pose> poses;
    //     for (auto ac_state : *detected_ac_states){
    //         geometry_msgs::Pose p;
    //         p = ac_state.pose;

    //         m_util_setpoint.Transform(p, ac_state.id, m_big_marker_trans_param, m_medium_marker_trans_param, m_small_marker_trans_param);
    //         poses.push_back(p);
    //     }
    //     ac_states_msg.target_pose = m_util_setpoint.GetSetpoint(poses);
    // }

    // if (fitting_plane_msg.medium_is_valid || fitting_plane_msg.big_is_valid || fitting_plane_msg.small_is_valid){
    //     m_fitting_plane_pub.publish(fitting_plane_msg);
    // }
}

bool ArucoTracking::GetTransformation(const vector<vector<Point2f>> corners, const vector<int> ids, const vector<Vec3d> rvecs, 
    const vector<Vec3d> tvecs, int2pose& int_to_pose, tf2_msgs::TFMessage& tf_msg_list)
{
    // Create and publish tf message for each marker
    for (auto i = 0; i < rvecs.size(); ++i){
        
        auto it = find(m_marker_ids.begin(), m_marker_ids.end(), ids[i]);
        if (it == m_marker_ids.end()){
            continue;
        }

        geometry_msgs::TransformStamped aruco_tf_stamped;
        aruco_tf_stamped.header.stamp = ros::Time::now();
        aruco_tf_stamped.header.frame_id = m_camera_frame_id_param;

        auto translation_vector = tvecs[i];
        auto rotation_vector = rvecs[i];
        auto transform = m_util_marker.CreateTransform(translation_vector, rotation_vector);
        stringstream ss;
        string marker_tf_prefix = "marker";
        ss << marker_tf_prefix << ids[i];
        aruco_tf_stamped.child_frame_id = ss.str();
        aruco_tf_stamped.transform.translation.x = transform.getOrigin().getX();
        aruco_tf_stamped.transform.translation.y = transform.getOrigin().getY();
        aruco_tf_stamped.transform.translation.z = transform.getOrigin().getZ();

        auto q = m_util_marker.ZProjection(transform.getRotation().getX(),
                                            transform.getRotation().getY(),
                                            transform.getRotation().getZ(),
                                            transform.getRotation().getW());
        aruco_tf_stamped.transform.rotation = q;
        tf_msg_list.transforms.push_back(aruco_tf_stamped);

        geometry_msgs::Pose pose;
        pose.position.x = aruco_tf_stamped.transform.translation.x;
        pose.position.y = aruco_tf_stamped.transform.translation.y;
        pose.position.z = aruco_tf_stamped.transform.translation.z;
        pose.orientation = q;
        int_to_pose.insert(pair<int,geometry_msgs::Pose>(ids[i], pose));
    }

    return true;
}

void ArucoTracking::ImagePub(Mat image, const vector < vector<vector<Point2f>> > corners_vec, const vector < vector<int> > ids_vec)
{
    vector<vector<Point2f>> d_corners;
    for (auto corners : corners_vec){
        d_corners.insert(d_corners.end(), corners.begin(), corners.end());
    }

    vector<int> d_ids;
    for (auto ids : ids_vec){
        d_ids.insert(d_ids.end(), ids.begin(), ids.end());
    }
  
    Mat copy_image;
    image.copyTo(copy_image);
    aruco::drawDetectedMarkers(copy_image, d_corners, d_ids);

    sensor_msgs::Image img_msg; // >> message to be sent
    m_cv_ptr->image = copy_image;
    m_cv_ptr->header.frame_id = m_camera_frame_id_param;
    img_msg = *m_cv_ptr->toImageMsg();
    m_cv_ptr = nullptr;
    m_image_pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
}

void ArucoTracking::TargetPub(kuam_msgs::ArucoStates ac_states_msg, kuam_msgs::ArucoVisuals ac_visuals_msg)
{
    m_target_state_pub.publish(ac_states_msg);
    m_visual_pub.publish(ac_visuals_msg);
}
}