#include <kuam_aruco_tracking/aruco_tracking.h>

#include <geometry_msgs/Point.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/Int16.h>
#include <kuam_msgs/ArucoVisual.h>

using namespace std;
using namespace cv;

namespace kuam{
    
ArucoTracking::ArucoTracking() :
    OPENCV_WINDOW("Image window"),
    m_usb_cam_logging_topic_param("missing"),
    m_calib_path_param("missing"),
    m_detector_params_path_param("missing"),
    m_camera_frame_id_param("missing"),
    m_process_freq_param(NAN),
    m_dictionaryID_param(NAN),
    m_big_marker_id_param(NAN),
    m_small_marker_id_param(NAN),
    m_big_marker_size_m_param(NAN),
    m_small_marker_size_m_param(NAN),
    m_filter_buf_size_param(NAN),
    m_estimating_method_param(NAN),
    m_noise_dist_th_m_param(NAN),
    m_noise_cnt_th_param(NAN),
    m_cv_ptr(nullptr),
    MARKER_ID_STACK_SIZE(30),
    m_marker_cnt_th_param(NAN)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("[aruco_tracking] Fail GetParam");
    InitROS();

    //namedWindow(OPENCV_WINDOW);
    m_parser.ReadFile(m_dictionaryID_param, m_calib_path_param, m_detector_params_path_param,
                    m_detector_params, m_dictionary, m_cam_matrix, m_dist_coeffs);

    m_target.state.resize((int)EstimatingMethod::ItemNum);
    m_target.last_detected_time = ros::Time(0);
    m_target_pose_list.header.frame_id = m_camera_frame_id_param;

    m_id_to_markersize_map.insert(pair<int, float>(m_big_marker_id_param, m_big_marker_size_m_param));
    m_id_to_markersize_map.insert(pair<int, float>(m_small_marker_id_param, m_small_marker_size_m_param));
}

ArucoTracking::~ArucoTracking()
{
    //destroyWindow(OPENCV_WINDOW);
}

bool ArucoTracking::InitFlag()
{
    m_target.is_detected = false;
    m_target.is_init = false;
    m_fix_small_marker = false;
    return true;
}

bool ArucoTracking::GetParam()
{
    string nd_name = ros::this_node::getName();
    string ns_name = ros::this_node::getNamespace();

    m_nh.getParam(nd_name + "/usb_cam_logging_topic", m_usb_cam_logging_topic_param);
    m_nh.getParam(nd_name + "/calib_path", m_calib_path_param);
    m_nh.getParam(nd_name + "/detector_params_path", m_detector_params_path_param);
    m_nh.getParam(ns_name + "/usb_cam/camera_frame_id", m_camera_frame_id_param);
    m_nh.getParam(nd_name + "/is_eval", m_is_eval_param);
    m_nh.getParam(nd_name + "/compare_mode", m_compare_mode_param);
    m_nh.getParam(nd_name + "/using_gazebo_data", m_using_gazebo_data_param);
    m_nh.getParam(nd_name + "/using_logging_data", m_using_logging_data_param);
    m_nh.getParam(nd_name + "/dictionaryID", m_dictionaryID_param);
    m_nh.getParam(nd_name + "/big_marker_id", m_big_marker_id_param);
    m_nh.getParam(nd_name + "/small_marker_id", m_small_marker_id_param);
    m_nh.getParam(nd_name + "/big_marker_size_m", m_big_marker_size_m_param);
    m_nh.getParam(nd_name + "/small_marker_size_m", m_small_marker_size_m_param);
    m_nh.getParam(nd_name + "/filter_buf_size", m_filter_buf_size_param);
    m_nh.getParam(nd_name + "/estimating_method", m_estimating_method_param);
    m_nh.getParam(nd_name + "/noise_dist_th_m", m_noise_dist_th_m_param);
    m_nh.getParam(nd_name + "/noise_cnt_th", m_noise_cnt_th_param);
    m_nh.getParam(nd_name + "/process_freq", m_process_freq_param);
    m_nh.getParam(nd_name + "/marker_cnt_th", m_marker_cnt_th_param);

    if (m_calib_path_param == "missing") { ROS_ERROR_STREAM("[aruco_tracking] m_calib_path_param is missing"); return false; }
    else if (m_detector_params_path_param == "missing") { ROS_ERROR_STREAM("[aruco_tracking] m_detector_params_path_param is missing"); return false; }
    else if (m_usb_cam_logging_topic_param == "missing") { ROS_ERROR_STREAM("[aruco_tracking] m_usb_cam_logging_topic_param is missing"); return false; }
    else if (m_camera_frame_id_param == "missing") { ROS_ERROR_STREAM("[aruco_tracking] m_camera_frame_id_param is missing"); return false; }
    else if (__isnan(m_dictionaryID_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_dictionaryID_param is NAN"); return false; }
    else if (__isnan(m_big_marker_id_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_big_marker_id_param is NAN"); return false; }
    else if (__isnan(m_small_marker_id_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_small_marker_id_param is NAN"); return false; }
    else if (__isnan(m_big_marker_size_m_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_big_marker_size_m_param is NAN"); return false; }
    else if (__isnan(m_small_marker_size_m_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_small_marker_size_m_param is NAN"); return false; }
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
    string nd_name = ros::this_node::getName();
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
    m_image_pub = m_nh.advertise<sensor_msgs::Image>(nd_name + "/output_video", 10);
    m_tf_list_pub = m_nh.advertise<tf2_msgs::TFMessage>(nd_name + "/tf_list", 10);
    m_visual_pub = m_nh.advertise<kuam_msgs::ArucoVisual> (nd_name + "/aruco_visual", 1);
    m_target_state_pub = m_nh.advertise<kuam_msgs::ArucoState> (nd_name + "/target_state", 1);
    m_target_list_pub = m_nh.advertise<geometry_msgs::PoseArray> (nd_name + "/target_list", 1);
    if (m_is_eval_param){
        m_cnt_pub = m_nh.advertise<std_msgs::Int16> (nd_name + "/cnt", 1);
        m_target_marker_pub = m_nh.advertise<std_msgs::Int16> (nd_name + "/target_marker", 1);
    }
    
    // Initialize timer
    m_image_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &ArucoTracking::ProcessTimerCallback, this);

    return true;
}

void ArucoTracking::ProcessTimerCallback(const ros::TimerEvent& event)
{
    if (m_cv_ptr != nullptr){
        vector<int> ids;
        bool is_detected = false;
        geometry_msgs::Pose target_pose;
        MarkerPoseEstimating(ids, target_pose, is_detected);

        TargetStateEstimating(ids, target_pose, is_detected);
    }
}

void ArucoTracking::ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr)
{
    if(!Convert2CVImg(img_ptr)) ROS_ERROR("[aruco_tracking] Fail to convert sensor_msgs to cv_image");
}


bool ArucoTracking::MarkerPoseEstimating(vector<int>& ids, geometry_msgs::Pose& target_pose, bool& is_detected)
{
    static int waitTime = 0;
    static double totalTime = 0;
    static int totalIterations = 0;

    double tick = (double)getTickCount();

    vector<vector<Point2f>> corners;
    vector<vector<Point2f>> rejected;

    Mat image;
    image = m_cv_ptr->image;

    aruco::detectMarkers(image, m_dictionary, corners, ids, m_detector_params, rejected);

    bool is_enough = false;
    if (ids.size() > 0){
        // Shift detected marker stack vector
        if (m_detected_marker_num_stack.size() < MARKER_ID_STACK_SIZE){
            m_detected_marker_num_stack.push_back(ids.size());
        }
        else {
            int index;
            for (index = 0; index < MARKER_ID_STACK_SIZE - 1; index++){
                m_detected_marker_num_stack[index] = m_detected_marker_num_stack[index + 1];
            }
            m_detected_marker_num_stack[index] = ids.size();
        }

        // Count multi aruco marker detected in detected marker stack vector
        int cnt = 0;
        for (auto num : m_detected_marker_num_stack){
            if (num == 2) cnt++;
        }
        if (cnt > m_marker_cnt_th_param){
            is_enough = true;
        }

        // Timer for multi aruco marker detected
        static bool is_start_timer = false;
        if (is_enough){
            if (!is_start_timer){
                is_start_timer = true;
                m_last_enough_time = ros::Time::now();
            }
            if ((ros::Time::now() - m_last_enough_time) > ros::Duration(3)){
                m_fix_small_marker = true;
            }
            auto eclapse = ros::Time::now() - m_last_enough_time;
        }
        else{
            is_start_timer = false;
        }

        // For comparing detected marker count and ego vehicle height
        if (m_is_eval_param){
            std_msgs::Int16 msg;
            msg.data = cnt;
            m_cnt_pub.publish(msg);
        }
    }

    // Remove unused marker
    if (ids.size() > 0){
        if (m_fix_small_marker){
            unsigned int big_index;
            bool has_big = false;
            for (int index = 0; index < ids.size(); index++){
                if (ids[index] == m_big_marker_id_param){
                    big_index = index;
                    has_big = true;
                }
            }

            if (has_big){
                corners.erase(corners.begin() + big_index);
                ids.erase(ids.begin() + big_index);
            }
        }
        else{
            unsigned int small_index;
            bool has_small = false;
            for (int index = 0; index < ids.size(); index++){
                if (ids[index] == m_small_marker_id_param){
                    small_index = index;
                    has_small = true;
                }
            }

            if (has_small){
                corners.erase(corners.begin() + small_index);
                ids.erase(ids.begin() + small_index);
            }
        }
    }

    // Noise filter
    for (auto id : ids){
        if (m_id_to_markersize_map.find(id) != m_id_to_markersize_map.end()){
            if (IsNoise()) is_detected = false; 
            else{
                is_detected = true;
                m_target.id = id;
                m_target.marker_size_m = m_id_to_markersize_map.find(id)->second;

                if (m_is_eval_param){
                    std_msgs::Int16 msg;
                    msg.data = id;
                    m_target_marker_pub.publish(msg);
                }
            } 
        }
    }

    Mat copy_image;
    image.copyTo(copy_image);  
    vector<Vec3d> rvecs, tvecs;
    if (is_detected){
        aruco::estimatePoseSingleMarkers(corners, m_target.marker_size_m, m_cam_matrix, m_dist_coeffs, rvecs, tvecs);
        
        // if detected value is noise, change is_detected value from true to false
        Camera2World(corners, ids, rvecs, tvecs, target_pose);
        m_target_pose_list.poses.push_back(target_pose);
        m_target_list_pub.publish(m_target_pose_list);
        if (IsNoise(target_pose)) is_detected = false;
        else is_detected = true;

        // double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        // totalTime += currentTime;
        // totalIterations++;
        // if(totalIterations % 30 == 0) {
        //     ROS_INFO("Detection Time = %f ms (Mean = %f ms)", currentTime * 1000, 1000 * totalTime / double(totalIterations));
        // }

        // draw results
        
        if (is_detected){
            aruco::drawDetectedMarkers(copy_image, corners, ids);

            for (auto id : ids){
                if (m_id_to_markersize_map.find(id) != m_id_to_markersize_map.end()){
                    aruco::drawAxis(copy_image, m_cam_matrix, m_dist_coeffs, rvecs[id], tvecs[id],
                                    m_target.marker_size_m * 0.5f);
                }
            }
        }
    }

    static unsigned cnt = 0;

    if (is_detected){
        cnt++;

        if (cnt > 2){
            cnt = 0;
            sensor_msgs::Image img_msg; // >> message to be sent
            m_cv_ptr->image = copy_image;
            m_cv_ptr->header.frame_id = m_camera_frame_id_param;
            img_msg = *m_cv_ptr->toImageMsg();
            m_image_pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
        }
    }

    return true;
}

bool ArucoTracking::Camera2World(const vector<vector<Point2f>> corners, const vector<int> ids, const vector<Vec3d> rvecs, const vector<Vec3d> tvecs, geometry_msgs::Pose& target_pose)
{
    // Create and publish tf message for each marker
    tf2_msgs::TFMessage tf_msg_list;
    for (auto i = 0; i < rvecs.size(); ++i){
        if (ids[i] == m_target.id){

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

            target_pose.position.x = transform.getOrigin().getX();
            target_pose.position.y = transform.getOrigin().getY();
            target_pose.position.z = transform.getOrigin().getZ();
            target_pose.orientation.x = transform.getRotation().getX();
            target_pose.orientation.y = transform.getRotation().getY();
            target_pose.orientation.z = transform.getRotation().getZ();
            target_pose.orientation.w = transform.getRotation().getW();
        }
    }
    m_tf_list_pub.publish(tf_msg_list);

    return true;
}

bool ArucoTracking::TargetStateEstimating(const vector<int> ids, const geometry_msgs::Pose pose, const bool is_detected)
{
    Eigen::Vector3d pos;
    pos << pose.position.x, pose.position.y, pose.position.z;

    if (is_detected){
        m_target.is_detected = true;
        m_last_detected_time = ros::Time::now();

        if (m_compare_mode_param){
            WithoutFilter(pos, m_target.state[(int)EstimatingMethod::WOF]);
            MovingAvgFilter(pos, m_target.state[(int)EstimatingMethod::MAF]);
            ExpMovingAvgFilter(pos, m_target.state[(int)EstimatingMethod::EMAF]);
        }
        else {
            switch (m_estimating_method_param){
            case (int)EstimatingMethod::WOF:
                WithoutFilter(pos, m_target.state[(int)EstimatingMethod::WOF]);

                break;
            case (int)EstimatingMethod::MAF:
                MovingAvgFilter(pos, m_target.state[(int)EstimatingMethod::MAF]);

                break;
            case (int)EstimatingMethod::EMAF:
                ExpMovingAvgFilter(pos, m_target.state[(int)EstimatingMethod::EMAF]);

                break;
            default:
                break;
            }
        }
    }
    else{
        if ((ros::Time::now() - m_last_detected_time) > ros::Duration(0.5)){
            m_target.id = -1;
            m_target.is_detected = false;
        }
        else{
            if (m_compare_mode_param){
                WithoutFilter(m_target.state[(int)EstimatingMethod::WOF].prev_position, m_target.state[(int)EstimatingMethod::WOF]);
                MovingAvgFilter(m_target.state[(int)EstimatingMethod::MAF].prev_position, m_target.state[(int)EstimatingMethod::MAF]);
                ExpMovingAvgFilter(m_target.state[(int)EstimatingMethod::EMAF].prev_position, m_target.state[(int)EstimatingMethod::EMAF]);
            }
            else {
                switch (m_estimating_method_param){
                case (int)EstimatingMethod::WOF:
                    WithoutFilter(m_target.state[(int)EstimatingMethod::WOF].prev_position, m_target.state[(int)EstimatingMethod::WOF]);

                    break;
                case (int)EstimatingMethod::MAF:
                    MovingAvgFilter(m_target.state[(int)EstimatingMethod::MAF].prev_position, m_target.state[(int)EstimatingMethod::MAF]);

                    break;
                case (int)EstimatingMethod::EMAF:
                    ExpMovingAvgFilter(m_target.state[(int)EstimatingMethod::EMAF].prev_position, m_target.state[(int)EstimatingMethod::EMAF]);

                    break;
                default:
                    break;
                }
            }
        }
    }

    static unsigned int cnt = 0;
    kuam_msgs::ArucoState target_msg;
    target_msg.header.frame_id = m_camera_frame_id_param;
    target_msg.header.seq = cnt++;
    target_msg.header.stamp = ros::Time::now();
    target_msg.is_detected = m_target.is_detected;
    target_msg.id = m_target.id;

    switch (m_estimating_method_param){
        case (int)EstimatingMethod::WOF:
            target_msg.pose.position.x = m_target.state[(int)EstimatingMethod::WOF].position.x();
            target_msg.pose.position.y = m_target.state[(int)EstimatingMethod::WOF].position.y();
            target_msg.pose.position.z = m_target.state[(int)EstimatingMethod::WOF].position.z();

            break;
        case (int)EstimatingMethod::MAF:
            target_msg.pose.position.x = m_target.state[(int)EstimatingMethod::MAF].position.x();
            target_msg.pose.position.y = m_target.state[(int)EstimatingMethod::MAF].position.y();
            target_msg.pose.position.z = m_target.state[(int)EstimatingMethod::MAF].position.z();

            break;
        case (int)EstimatingMethod::EMAF:
            target_msg.pose.position.x = m_target.state[(int)EstimatingMethod::EMAF].position.x();
            target_msg.pose.position.y = m_target.state[(int)EstimatingMethod::EMAF].position.y();
            target_msg.pose.position.z = m_target.state[(int)EstimatingMethod::EMAF].position.z();

            break;
        default:
            break;
    }
    m_target_state_pub.publish(target_msg);


    static unsigned int cnt2 = 0;
    kuam_msgs::ArucoVisual aruco_visual_msg;
    aruco_visual_msg.header.frame_id = m_camera_frame_id_param;
    aruco_visual_msg.header.seq = cnt2++;
    aruco_visual_msg.header.stamp = ros::Time::now();

    aruco_visual_msg.is_detected = m_target.is_detected;
    aruco_visual_msg.is_compare_mode = m_compare_mode_param;
    aruco_visual_msg.id = m_target.id;
    aruco_visual_msg.esti_method =  m_estimating_method_param;
    for (auto state : m_target.state){
        geometry_msgs::Pose pose;
        pose.position.x = state.position.x();
        pose.position.y = state.position.y();
        pose.position.z = state.position.z();
        aruco_visual_msg.poses.push_back(pose);
    }
    m_visual_pub.publish(aruco_visual_msg);
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

bool ArucoTracking::WithoutFilter(const Eigen::Vector3d pos, State& state)
{
    state.position = pos;
    state.prev_position = pos;

    m_target.is_init = true;
}

bool ArucoTracking::MovingAvgFilter(const Eigen::Vector3d pos, State& state)
{
    static bool is_init = false;
    static vector<Eigen::Vector3d> pos_buf;

    // Initialize
    if (!is_init){
        pos_buf.resize(m_filter_buf_size_param);
        for (auto p : pos_buf){
            p = pos;
        }

        is_init = true;
    }

    // Shift
    for (int i = 0; i < m_filter_buf_size_param - 1; i++){
        pos_buf.at(i) = pos_buf.at(i+1);
    }
    pos_buf.at(m_filter_buf_size_param - 1) = pos;

    // Summation
    Eigen::Vector3d sum;
    sum << 0, 0, 0;
    for (auto p : pos_buf){
        sum += p;
    }

    // Average
    state.position = sum/(double)m_filter_buf_size_param;
    state.prev_position = pos;

    m_target.is_init = true;
}

bool ArucoTracking::ExpMovingAvgFilter(const Eigen::Vector3d pos, State& state)
{
    static bool is_init = false;
    static double w = 0.0;

    // Initialize
    if (!is_init){
        w = 2.0 / ((double)m_filter_buf_size_param + 1.0);
        
        state.prev_position = pos;
        
        is_init = true;
    }

    // exponential moving average filter
    state.position = w*pos + (1.0 - w)*state.prev_position;
    state.prev_position = pos;

    m_target.is_init = true;
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

bool ArucoTracking::IsNoise()
{
    static bool is_init = false;
    static unsigned int cnt = 0;

    if (!m_target.is_detected){
        if (!is_init){
            is_init = true;
            cnt = 0;
            m_last_noise_check_time = ros::Time::now();
        }
        
        if ((ros::Time::now() - m_last_noise_check_time) < ros::Duration(1.0)){
            cnt++;
            return true;
        }
        else {
            is_init = false;
            
            if (cnt >= m_noise_cnt_th_param) return false;
            else return true;
        }
    }
    else {
        return false;
        return !m_target.is_detected;
    }
}

bool ArucoTracking::IsNoise(const geometry_msgs::Pose target_pose)
{
    if (!m_target.is_detected){
        return false;
    }
    else {
        geometry_msgs::Point prev_pos;
        geometry_msgs::Point cur_pos = target_pose.position;

        switch (m_estimating_method_param){
        case (int)EstimatingMethod::WOF:
            prev_pos.x = m_target.state[(int)EstimatingMethod::WOF].prev_position.x();
            prev_pos.y = m_target.state[(int)EstimatingMethod::WOF].prev_position.y();
            prev_pos.z = m_target.state[(int)EstimatingMethod::WOF].prev_position.z();

            break;
        case (int)EstimatingMethod::MAF:
            prev_pos.x = m_target.state[(int)EstimatingMethod::MAF].prev_position.x();
            prev_pos.y = m_target.state[(int)EstimatingMethod::MAF].prev_position.y();
            prev_pos.z = m_target.state[(int)EstimatingMethod::MAF].prev_position.z();

            break;
        case (int)EstimatingMethod::EMAF:
            prev_pos.x = m_target.state[(int)EstimatingMethod::EMAF].prev_position.x();
            prev_pos.y = m_target.state[(int)EstimatingMethod::EMAF].prev_position.y();
            prev_pos.z = m_target.state[(int)EstimatingMethod::EMAF].prev_position.z();

            break;
        default:
            break;
        }

        auto dist = m_utils.Distance3D(prev_pos, cur_pos);
        
        if (dist > m_noise_dist_th_m_param) return true;
        else return false;
    }
}
}
