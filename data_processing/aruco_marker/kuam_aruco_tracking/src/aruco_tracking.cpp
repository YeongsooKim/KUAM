#include <kuam_aruco_tracking/aruco_tracking.h>

#include <geometry_msgs/Point.h>

#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <kuam_msgs/ArucoVisual.h>
#include <kuam_msgs/ArucoVisuals.h>
#include <kuam_msgs/ArucoStates.h>
#include <algorithm>

using namespace std;
using namespace cv;

namespace kuam{
    
ArucoTracking::ArucoTracking() :
    m_p_nh("~"),
    OPENCV_WINDOW("Image window"),
    m_usb_cam_logging_topic_param("missing"),
    m_calib_path_param("missing"),
    m_detector_params_path_param("missing"),
    m_camera_frame_id_param("missing"),
    m_process_freq_param(NAN),
    m_dictionaryID_param(NAN),
    m_big_marker_size_m_param(NAN),
    m_medium_marker_size_m_param(NAN),
    m_filter_buf_size_param(NAN),
    m_estimating_method_param(NAN),
    m_noise_dist_th_m_param(NAN),
    m_noise_cnt_th_param(NAN),
    m_cv_ptr(nullptr),
    MARKER_ID_QUEUE_SIZE(30),
    m_marker_cnt_th_param(NAN)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("[aruco_tracking] Fail GetParam");
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
    return true;
}

bool ArucoTracking::GetParam()
{
    string ns_name = ros::this_node::getNamespace();

    m_p_nh.getParam("usb_cam_logging_topic", m_usb_cam_logging_topic_param);
    m_p_nh.getParam("calib_path", m_calib_path_param);
    m_p_nh.getParam("detector_params_path", m_detector_params_path_param);
    m_nh.getParam(ns_name + "/usb_cam/camera_frame_id", m_camera_frame_id_param);
    m_p_nh.getParam("compare_mode", m_compare_mode_param);
    m_p_nh.getParam("using_gazebo_data", m_using_gazebo_data_param);
    m_p_nh.getParam("using_logging_data", m_using_logging_data_param);
    m_p_nh.getParam("dictionaryID", m_dictionaryID_param);
    m_p_nh.getParam("big_marker_size_m", m_big_marker_size_m_param);
    m_p_nh.getParam("medium_marker_size_m", m_medium_marker_size_m_param);
    m_p_nh.getParam("small_marker_size_m", m_small_marker_size_m_param);
    m_p_nh.getParam("filter_buf_size", m_filter_buf_size_param);
    m_p_nh.getParam("estimating_method", m_estimating_method_param);
    m_p_nh.getParam("noise_dist_th_m", m_noise_dist_th_m_param);
    m_p_nh.getParam("noise_cnt_th", m_noise_cnt_th_param);
    m_p_nh.getParam("process_freq", m_process_freq_param);
    m_p_nh.getParam("marker_cnt_th", m_marker_cnt_th_param);

    XmlRpc::XmlRpcValue list;
    m_p_nh.getParam("big_marker_ids", list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < list.size(); ++i) {
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        int id = static_cast<int>(list[i]);
        m_big_marker_ids_param.push_back(id);
    }
    m_p_nh.getParam("medium_marker_ids", list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < list.size(); ++i) {
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        int id = static_cast<int>(list[i]);
        m_medium_marker_ids_param.push_back(id);
    }

    m_p_nh.getParam("small_marker_ids", list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < list.size(); ++i) {
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        int id = static_cast<int>(list[i]);
        m_small_marker_ids_param.push_back(id);
    }

    if (m_calib_path_param == "missing") { ROS_ERROR_STREAM("[aruco_tracking] m_calib_path_param is missing"); return false; }
    else if (m_detector_params_path_param == "missing") { ROS_ERROR_STREAM("[aruco_tracking] m_detector_params_path_param is missing"); return false; }
    else if (m_usb_cam_logging_topic_param == "missing") { ROS_ERROR_STREAM("[aruco_tracking] m_usb_cam_logging_topic_param is missing"); return false; }
    else if (m_camera_frame_id_param == "missing") { ROS_ERROR_STREAM("[aruco_tracking] m_camera_frame_id_param is missing"); return false; }
    else if (__isnan(m_dictionaryID_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_dictionaryID_param is NAN"); return false; }
    else if (__isnan(m_big_marker_size_m_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_big_marker_size_m_param is NAN"); return false; }
    else if (__isnan(m_medium_marker_size_m_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_medium_marker_size_m_param is NAN"); return false; }
    else if (__isnan(m_filter_buf_size_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_filter_buf_size_param is NAN"); return false; }
    else if (__isnan(m_estimating_method_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_estimating_method_param is NAN"); return false; }
    else if (__isnan(m_noise_dist_th_m_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_noise_dist_th_m_param is NAN"); return false; }
    else if (__isnan(m_noise_cnt_th_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_noise_cnt_th_param is NAN"); return false; }
    else if (__isnan(m_process_freq_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_process_freq_param is NAN"); return false; }
    else if (__isnan(m_marker_cnt_th_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_marker_cnt_th_param is NAN"); return false; }

    return true;
}

bool ArucoTracking::InitROS()
{
    // package, node, topic name
    string ns_name = ros::this_node::getNamespace();
    string image_topic_name;
    if (m_using_gazebo_data_param) image_topic_name = "/camera/rgb/image_raw";
    else {
        if (m_using_logging_data_param) image_topic_name = m_usb_cam_logging_topic_param;
        else image_topic_name = ns_name + "/usb_cam/image_rect_color";
    } 

    // Initialize subscriber
    m_image_sub = m_nh.subscribe<sensor_msgs::Image>(image_topic_name, 1, boost::bind(&ArucoTracking::ImageCallback, this, _1));
    
    // Initialize publisher
    m_image_pub = m_p_nh.advertise<sensor_msgs::Image>("output_video", 10);
    m_tf_list_pub = m_p_nh.advertise<tf2_msgs::TFMessage>("tf_list", 10);
    m_visual_pub = m_p_nh.advertise<kuam_msgs::ArucoVisuals> ("aruco_visuals", 1);
    m_target_state_pub = m_p_nh.advertise<kuam_msgs::ArucoStates> ("target_states", 1);
    
    // Initialize timer
    m_image_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &ArucoTracking::ProcessTimerCallback, this);

    return true;
}

bool ArucoTracking::InitMarker()
{
    for (auto id : m_big_marker_ids_param){
        Target target(id, m_big_marker_size_m_param, m_filter_buf_size_param, m_noise_cnt_th_param, 
            m_noise_dist_th_m_param, m_estimating_method_param, m_compare_mode_param);
        
        m_targets.push_back(target);
        m_marker_ids.push_back(id);
    }
    for (auto id : m_medium_marker_ids_param){
        Target target(id, m_medium_marker_size_m_param, m_filter_buf_size_param, m_noise_cnt_th_param, 
            m_noise_dist_th_m_param, m_estimating_method_param, m_compare_mode_param);
        
        m_targets.push_back(target);
        m_marker_ids.push_back(id);
    }
    for (auto id : m_small_marker_ids_param){
        Target target(id, m_small_marker_size_m_param, m_filter_buf_size_param, m_noise_cnt_th_param, 
            m_noise_dist_th_m_param, m_estimating_method_param, m_compare_mode_param);
        
        m_targets.push_back(target);
        m_marker_ids.push_back(id);
    }
}

void ArucoTracking::ProcessTimerCallback(const ros::TimerEvent& event)
{
    if (m_cv_ptr == nullptr){
        return;
    }

    Mat image;
    image = m_cv_ptr->image;
    image = image(Rect(40, 40, 600, 440));
    vector<int> ids;
    vector<vector<Point2f>> corners;
    vector<vector<Point2f>> rejected;
    aruco::detectMarkers(image, m_dictionary, corners, ids, m_detector_params, rejected);

    vector<int> s_ids, m_ids, b_ids;
    vector<vector<Point2f>> s_corners, m_corners, b_corners;
    SelectMarkers(corners, ids, s_corners, s_ids, m_corners, m_ids, b_corners, b_ids);

    NoiseFilter(s_corners, s_ids, m_corners, m_ids, b_corners, b_ids);
    
    if (s_ids.empty() && m_ids.empty() && b_ids.empty()){
        return;
    }
  
    int2pose int_to_pose;
    tf2_msgs::TFMessage tf_msg_list;

    vector<Vec3d> b_rvecs, b_tvecs;
    aruco::estimatePoseSingleMarkers(b_corners, m_big_marker_size_m_param, m_cam_matrix, m_dist_coeffs, b_rvecs, b_tvecs);
    Camera2World(b_corners, b_ids, b_rvecs, b_tvecs, int_to_pose, tf_msg_list);

    vector<Vec3d> m_rvecs, m_tvecs;
    aruco::estimatePoseSingleMarkers(m_corners, m_medium_marker_size_m_param, m_cam_matrix, m_dist_coeffs, m_rvecs, m_tvecs);
    Camera2World(m_corners, m_ids, m_rvecs, m_tvecs, int_to_pose, tf_msg_list);

    vector<Vec3d> s_rvecs, s_tvecs;
    aruco::estimatePoseSingleMarkers(s_corners, m_small_marker_size_m_param, m_cam_matrix, m_dist_coeffs, s_rvecs, s_tvecs);
    Camera2World(s_corners, s_ids, s_rvecs, s_tvecs, int_to_pose, tf_msg_list);

    MarkerUpdate(b_ids, m_ids, s_ids, int_to_pose);

    m_tf_list_pub.publish(tf_msg_list);
    ImagePub(image, s_corners, s_ids, m_corners, m_ids, b_corners, b_ids);
    TargetPub();
}

void ArucoTracking::ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr)
{
    if(!Convert2CVImg(img_ptr)) ROS_ERROR("[aruco_tracking] Fail to convert sensor_msgs to cv_image");
}

void ArucoTracking::SelectMarkers(vector<vector<Point2f>>& corners, vector<int>& ids,
    vector<vector<Point2f>>& s_corners, vector<int>& s_ids,
    vector<vector<Point2f>>& m_corners, vector<int>& m_ids,
    vector<vector<Point2f>>& b_corners, vector<int>& b_ids)
{
    if (ids.empty()){
        return;
    }
    
    GetIdsnCorners(m_big_marker_ids_param, ids, corners, b_ids, b_corners);
    EraseIdnCorner(b_ids, ids, corners);

    GetIdsnCorners(m_medium_marker_ids_param, ids, corners, m_ids, m_corners);
    EraseIdnCorner(m_ids, ids, corners);
    
    GetIdsnCorners(m_small_marker_ids_param, ids, corners, s_ids, s_corners);
    EraseIdnCorner(s_ids, ids, corners);
}

void ArucoTracking::NoiseFilter(vector<vector<Point2f>>& s_corners, vector<int>& s_ids,
                                vector<vector<Point2f>>& m_corners, vector<int>& m_ids,
                                vector<vector<Point2f>>& b_corners, vector<int>& b_ids)
{
    vector<int> b_noises;
    GetNoiseIndexes(b_noises, b_ids);
    EraseIdnCorner(b_noises, b_ids, b_corners);

    vector<int> m_noises;
    GetNoiseIndexes(m_noises, m_ids);
    EraseIdnCorner(m_noises, m_ids, m_corners);

    vector<int> s_noises;
    GetNoiseIndexes(s_noises, s_ids);
    EraseIdnCorner(s_noises, s_ids, s_corners);
}

void ArucoTracking::MarkerUpdate(const vector<int> b_ids, const vector<int> m_ids, const vector<int> s_ids, int2pose int_to_pose)
{
    vector<int> detected_ids;
    vector<int> undetected_ids;
    GetDetectedId(detected_ids, undetected_ids, b_ids, m_big_marker_ids_param);
    GetDetectedId(detected_ids, undetected_ids, m_ids, m_medium_marker_ids_param);
    GetDetectedId(detected_ids, undetected_ids, s_ids, m_small_marker_ids_param);

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

bool ArucoTracking::Camera2World(const vector<vector<Point2f>> corners, const vector<int> ids, const vector<Vec3d> rvecs, 
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
        auto transform = CreateTransform(translation_vector, rotation_vector);
        stringstream ss;
        string marker_tf_prefix = "marker";
        ss << marker_tf_prefix << ids[i];
        aruco_tf_stamped.child_frame_id = ss.str();
        aruco_tf_stamped.transform.translation.x = transform.getOrigin().getX();
        aruco_tf_stamped.transform.translation.y = transform.getOrigin().getY();
        aruco_tf_stamped.transform.translation.z = transform.getOrigin().getZ();
        aruco_tf_stamped.transform.rotation.x = transform.getRotation().getX();
        aruco_tf_stamped.transform.rotation.y = transform.getRotation().getY();
        aruco_tf_stamped.transform.rotation.z = transform.getRotation().getZ();
        aruco_tf_stamped.transform.rotation.w = transform.getRotation().getW();
        tf_msg_list.transforms.push_back(aruco_tf_stamped);

        geometry_msgs::Pose pose;
        pose.position.x = transform.getOrigin().getX();
        pose.position.y = transform.getOrigin().getY();
        pose.position.z = transform.getOrigin().getZ();
        pose.orientation.x = transform.getRotation().getX();
        pose.orientation.y = transform.getRotation().getY();
        pose.orientation.z = transform.getRotation().getZ();
        pose.orientation.w = transform.getRotation().getW();
        int_to_pose.insert(pair<int,geometry_msgs::Pose>(ids[i], pose));
    }

    return true;
}

void ArucoTracking::ImagePub(Mat image, const vector<vector<Point2f>> s_corners, const vector<int> s_ids,
                                const vector<vector<Point2f>> m_corners, const vector<int> m_ids,
                                const vector<vector<Point2f>> b_corners, const vector<int> b_ids)
{
    vector<int> d_ids;
    d_ids.insert(d_ids.end(), b_ids.begin(), b_ids.end());
    d_ids.insert(d_ids.end(), m_ids.begin(), m_ids.end());
    d_ids.insert(d_ids.end(), s_ids.begin(), s_ids.end());

    vector<vector<Point2f>> d_corners;
    d_corners.insert(d_corners.end(), b_corners.begin(), b_corners.end());
    d_corners.insert(d_corners.end(), m_corners.begin(), m_corners.end());
    d_corners.insert(d_corners.end(), s_corners.begin(), s_corners.end());

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

void ArucoTracking::TargetPub()
{
    kuam_msgs::ArucoStates ac_states_msg;
    kuam_msgs::ArucoVisuals ac_visuals_msg;

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
    m_target_state_pub.publish(ac_states_msg);
    m_visual_pub.publish(ac_visuals_msg);
}

bool ArucoTracking::Convert2CVImg(const sensor_msgs::Image::ConstPtr &img_ptr)
{
    try {
        m_cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
        return true;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("[aruco_tracking] cv_bridge exception: %s", e.what());
        return false;
    }
}

tf2::Vector3 ArucoTracking::CvVector3d2TfVector3(const Vec3d &vec) 
{
    return {vec[0], vec[1], vec[2]};
}

tf2::Quaternion ArucoTracking::CvVector3d2TfQuarternion(const Vec3d &rotation_vector) 
{
    Mat rotation_matrix;
    auto ax = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;

    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);
    return q;
}

tf2::Transform ArucoTracking::CreateTransform(const Vec3d &translation_vector, const Vec3d &rotation_vector) 
{
    tf2::Transform transform;
    transform.setOrigin(CvVector3d2TfVector3(translation_vector));
    transform.setRotation(CvVector3d2TfQuarternion(rotation_vector));
    return transform;
}

void ArucoTracking::GetDetectedId(vector<int>& detected_ids, vector<int>& undetected_ids,
                    const vector<int> src_ids, const vector<int> trg_ids)
{
    for (auto trg_id : trg_ids){
        auto it = find(src_ids.begin(), src_ids.end(), trg_id);
        if (it == src_ids.end()){
            undetected_ids.push_back(trg_id);
        }
        else{
            detected_ids.push_back(trg_id);
        }
    }    
}

void ArucoTracking::GetIdsnCorners(const vector<int> ref_ids, const vector<int> input_ids, const vector<vector<Point2f>> input_corners,
                            vector<int>& output_ids, vector<vector<Point2f>>& output_corners)
{
    for (int index = 0; index < input_ids.size(); index++){
        auto it = find(ref_ids.begin(), ref_ids.end(), input_ids[index]);
        if (it == ref_ids.end()){
            continue;
        }

        output_corners.push_back(input_corners[index]);
        output_ids.push_back(input_ids[index]);
    }
}

void ArucoTracking::GetNoiseIndexes(vector<int>& noises, const vector<int> ids)
{
    for (int index = 0; index < ids.size(); index++){
        for (auto& target : m_targets){
            if (target.GetId() != ids[index]){
                continue;
            }

            bool is_noise = target.CheckNoise();
            
            if (is_noise){
                noises.push_back(index);
            }
            break;
        }
    }
}

bool ArucoTracking::HasSmallMarker(const vector<int> ids)
{
    bool has_small = false;
    for (auto small_id : m_medium_marker_ids_param){
        if (!has_small){
            for (auto detected_id : ids){
                if (detected_id == small_id){
                    has_small = true;
                    break;
                }
            }
        }
    }
    return has_small;
}

void ArucoTracking::EraseIdnCorner(const vector<int> erase_ids, vector<int>& ids, vector<vector<Point2f>>& corners)
{
    vector<int> indexes;
    for (auto erase_id : erase_ids){
        for (int index = 0; index < ids.size(); index++){
            if (ids[index] == erase_id){
                indexes.push_back(index);
                break;
            }
        }
    }

    if (!indexes.empty()){
        sort(indexes.begin(), indexes.end(), greater<int>());

        for (auto index : indexes){
            corners.erase(corners.begin() + index);
            ids.erase(ids.begin() + index);
        }
    }
}

void ArucoTracking::KillMarker(const vector<int> kill_ids)
{
    for (auto id : kill_ids){
        for (auto& target : m_targets){
            if (target.GetId() == id){
                target.SetIsDetected(false);
                break;
            }
        }
    }
}
}