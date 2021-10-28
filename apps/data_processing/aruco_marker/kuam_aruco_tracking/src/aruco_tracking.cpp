#include <kuam_aruco_tracking/aruco_tracking.h>

#include <geometry_msgs/Point.h>

#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <kuam_msgs/ArucoVisual.h>
#include <kuam_msgs/ArucoVisuals.h>
#include <kuam_msgs/marker_ids.h>
#include <algorithm>

#include <kuam_msgs/FittingPlane.h>
#include <kuam_msgs/FittingPlanes.h>

using namespace std;
using namespace cv;

int FrequencyDegree::valid_id = -1;

namespace kuam{
    
ArucoTracking::ArucoTracking() :
    m_p_nh("~"),
    MARKER_ID_QUEUE_SIZE(30)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[aruco_tracking] Fail GetParam %s", m_err_param.c_str());
    InitROS();
    InitMarker();

    m_parser.ReadFile(m_dictionaryID_param, m_calib_path_param, m_detector_params_path_param,
                    m_detector_params, m_dictionary, m_cam_matrix, m_dist_coeffs);
}

ArucoTracking::~ArucoTracking()
{ }

bool ArucoTracking::InitFlag()
{
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
    if (!m_p_nh.getParam("plane_threshold", m_plane_threshold_param))  { m_err_param = "plane_threshold"; return false; }
    if (!m_p_nh.getParam("plane_iterations", m_plane_iterations_param))  { m_err_param = "plane_iterations"; return false; }
    if (!m_p_nh.getParam("angle_deg_threshold", m_angle_deg_threshold_param))  { m_err_param = "angle_deg_threshold"; return false; }

    if (!m_p_nh.getParam("marker_size_type_num", m_marker_size_type_num_param))  { m_err_param = "marker_size_type_num"; return false; }
    if (!m_p_nh.getParam("big_marker_trans", m_big_marker_trans_param))  { m_err_param = "big_marker_trans"; return false; }
    if (!m_p_nh.getParam("medium_marker_trans", m_medium_marker_trans_param))  { m_err_param = "medium_marker_trans"; return false; }
    if (!m_p_nh.getParam("small_marker_trans", m_small_marker_trans_param))  { m_err_param = "small_marker_trans"; return false; }
    
    if (!m_p_nh.getParam("dft_buf_size", m_dft_buf_size_param))  { m_err_param = "dft_buf_size"; return false; }
    if (!m_p_nh.getParam("frequency_degree_buf_size", m_freq_deg_buf_size_param))  { m_err_param = "frequency_degree_buf_size"; return false; }
    if (!m_p_nh.getParam("dft_integral_start_point", m_dft_integral_start_point_param))  { m_err_param = "freq_threshold"; return false; }
    if (!m_p_nh.getParam("frequency_degree_threshold", m_freq_degree_th_param))  { m_err_param = "frequency_degree_threshold"; return false; }
    if (!m_p_nh.getParam("difference_threshold_m", m_diff_th_m_param))  { m_err_param = "difference_threshold_m"; return false; }


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
    m_fitting_planes_pub = m_p_nh.advertise<kuam_msgs::FittingPlanes> ("fitting_planes", 1);
    
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


    // Update marker pose (one of three method, no filter, average fitler, exponential average filter)
    TargetStateEstimating(discrete_ids_vec, int_to_pose);


    // Calculate frequency degree of each marker
    CalFrequencyDegree();


    // // Plane Fitting
    kuam_msgs::FittingPlanes fitting_planes;
    std::shared_ptr<vector<kuam_msgs::ArucoState>> markers_contained_valid_plane = make_shared<vector<kuam_msgs::ArucoState>>();
    PlaneFitting(markers_contained_valid_plane, fitting_planes, discrete_ids_vec);


    // Get marker messages
    kuam_msgs::ArucoStates ac_states_msg;
    kuam_msgs::ArucoVisuals ac_visuals_msg;
    SetArucoMessages(ac_states_msg, ac_visuals_msg, markers_contained_valid_plane);


    // Publish
    m_tf_list_pub.publish(tf_msg_list);
    ImagePub(image, discrete_corners_vec, discrete_ids_vec);
    TargetPub(ac_states_msg, ac_visuals_msg);
    FittingPlanePub(fitting_planes);
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
            break;
        }
    }

    for (auto id : undetected_ids){
        for (auto& target : m_targets){
            if (target.GetId() != id){
                continue;
            }

            target.UpdateDetection(false);
            break;
        }
    }
}

void ArucoTracking::TargetStateEstimating(const vector < vector<int> > ids_vec, int2pose int_to_pose)
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

            target.TargetStateEstimating();
            break;
        }
    }
}

void ArucoTracking::CalFrequencyDegree()
{
    // Reverse sort
    auto all_marker_ids = m_marker_ids;
    std::sort(all_marker_ids.begin(), all_marker_ids.end(), std::greater<int>());

    for (auto id : all_marker_ids){
        bool is_valid = false;
        for (auto target : m_targets){
            if (target.GetId() != id) {
                continue;
            }

            // Is detected
            if (!target.GetIsDetected()){
                continue;
            }

            // Has frequency_degree of same id
            bool has_id;
            auto it = m_id2frequencyDegrees.find(target.GetId());
            if (it == m_id2frequencyDegrees.end()) has_id = false;
            else has_id = true;

            if (!has_id){
                FrequencyDegree frequency_degree(target.GetId(), m_dft_buf_size_param, m_freq_deg_buf_size_param);
                frequency_degree.Init(m_dft_integral_start_point_param, m_freq_degree_th_param,
                                    m_diff_th_m_param, target.GetZ(m_estimating_method_param));

                m_id2frequencyDegrees.insert(pair<int, FrequencyDegree> (target.GetId(), frequency_degree));
                
                continue;
            }

            // Calculate frequency degree of estimated pos
            it->second.CalEstimatedFreqDegree(target.GetZ(m_estimating_method_param));
            
            static double prev_freq_deg = 0.0;
            static bool is_first = true;
            if (it->second.IsValid()){
                is_valid = true;

                break;
            }
            else{
            }
        }

        if (is_valid){
            if (id > FrequencyDegree::valid_id){
                FrequencyDegree::valid_id = id;
                ROS_WARN("Set frequency degree valid id: %d", FrequencyDegree::valid_id);
            }
            break;
        }
    }
}


void ArucoTracking::PlaneFitting(std::shared_ptr<vector<kuam_msgs::ArucoState>>& markers_contained_valid_plane, 
                    kuam_msgs::FittingPlanes& fitting_planes, const vector < vector<int> > discrete_ids_vec)
{
    int plane_level = 0;
    for (auto ids : discrete_ids_vec){
        if (m_id2ZbtNormalAngles.find(plane_level) == m_id2ZbtNormalAngles.end()){
            Z2NormalAngle z2norm_angle;
            m_id2ZbtNormalAngles.insert(pair<int, Z2NormalAngle>(plane_level, z2norm_angle));
        }

        bool is_valid = false;
        geometry_msgs::PoseStamped plane_pose;
        auto it = m_id2ZbtNormalAngles.find(plane_level);
        vector<kuam_msgs::ArucoState> markers = m_util_marker.GetMarkerStates(ids, m_targets, 
                                            m_id2frequencyDegrees, m_camera_frame_id_param, m_estimating_method_param);

        if (m_util_setpoint.GeneratePlane(markers, plane_pose, it->second, m_plane_threshold_param, 
                                        m_plane_iterations_param, m_angle_deg_threshold_param, is_valid)){
            if (is_valid){
                markers_contained_valid_plane->insert(markers_contained_valid_plane->end(), markers.begin(), markers.end());
            }
            
            kuam_msgs::FittingPlane fitting_plane;
            fitting_plane.id = plane_level;
            fitting_plane.is_valid = true;
            fitting_plane.plane = plane_pose;
            fitting_plane.z_to_normal_deg = it->second.angle_deg;
            fitting_planes.planes.push_back(fitting_plane);
        }
        plane_level++;
    }
}


void ArucoTracking::SetArucoMessages(kuam_msgs::ArucoStates& ac_states_msg, 
        kuam_msgs::ArucoVisuals& ac_visuals_msg, std::shared_ptr<vector<kuam_msgs::ArucoState>>& markers_contained_valid_plane)
{
    ac_states_msg.header.frame_id = m_camera_frame_id_param;
    ac_states_msg.header.stamp = ros::Time::now();
    ac_states_msg.aruco_states = m_util_marker.GetMarkerStates(m_marker_ids, m_targets, m_id2frequencyDegrees, m_camera_frame_id_param, m_estimating_method_param);

    // Insert each detected aruco marker
    for (auto target : m_targets){
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

    // // Set target Pose
    // if (FrequencyDegree::valid_id != -1){
    //     for (auto aruco_state : ac_states_msg.aruco_states){
    //         if (aruco_state.id != FrequencyDegree::valid_id){
    //             continue;
    //         }

    //         if (aruco_state.is_detected){
    //             ac_states_msg.target_pose = aruco_state.pose;
    //             ac_states_msg.is_detected = true;
    //         }
    //     }
    // }

    // Set target pose
    if (!(*markers_contained_valid_plane).empty()){
        vector<geometry_msgs::Pose> poses;
        for (auto maker : *markers_contained_valid_plane){
            geometry_msgs::Pose p;
            p = maker.pose;

            m_util_setpoint.Transform(p, maker.id, m_big_marker_trans_param, m_medium_marker_trans_param, m_small_marker_trans_param);
            poses.push_back(p);
        }
        ac_states_msg.target_pose = m_util_setpoint.GetSetpoint(poses);
        ac_states_msg.is_detected = true;
    }
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

void ArucoTracking::FittingPlanePub(const kuam_msgs::FittingPlanes fitting_planes)
{
    bool is_valid = false;
    for (auto fitting_plane : fitting_planes.planes){
        if (fitting_plane.is_valid){
            is_valid = true;
        }
    }

    if (is_valid){
        m_fitting_planes_pub.publish(fitting_planes);
    }
}
}