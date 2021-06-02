#include <kuam_aruco_tracking/aruco_tracking.h>

#include <geometry_msgs/Point.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

using namespace std;
using namespace cv;

namespace kuam{
const string CAMERA_FRAME = "camera_link";

ArucoTracking::ArucoTracking() :
    m_process_freq_param(NAN),
    OPENCV_WINDOW("Image window"),
    m_aruco_parser_param("missing"),
    m_big_marker_id_param(NAN),
    m_small_marker_id_param(NAN),
    m_big_marker_size_m_param(NAN),
    m_small_marker_size_m_param(NAN),
    m_filter_buf_size_param(NAN),
    m_estimating_method_param(NAN),
    m_noise_dist_th_m_param(NAN),
    m_noise_cnt_th_param(NAN),
    m_cv_ptr(nullptr)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("[aruco_tracking] Fail GetParam");
    InitROS();
    InitMarkers();

    //namedWindow(OPENCV_WINDOW);
    m_parser.ReadFile(m_aruco_parser_param, m_do_estimate_pose, m_show_rejected, 
                    m_detector_params, m_dictionary, m_cam_matrix, m_dist_coeffs);
    
    m_target.state.resize((int)EstimatingMethod::ItemNum);
    m_target.last_detected_time = ros::Time(0);
    m_target_pose_list.header.frame_id = "camera_link";

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
    return true;
}

bool ArucoTracking::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_nh.getParam(nd_name + "/arcuo_parser", m_aruco_parser_param);
    m_nh.getParam(nd_name + "/big_marker_id", m_big_marker_id_param);
    m_nh.getParam(nd_name + "/small_marker_id", m_small_marker_id_param);
    m_nh.getParam(nd_name + "/big_marker_size_m", m_big_marker_size_m_param);
    m_nh.getParam(nd_name + "/small_marker_size_m", m_small_marker_size_m_param);
    m_nh.getParam(nd_name + "/filter_buf_size", m_filter_buf_size_param);
    m_nh.getParam(nd_name + "/estimating_method", m_estimating_method_param);
    m_nh.getParam(nd_name + "/compare_mode", m_compare_mode_param);
    m_nh.getParam(nd_name + "/using_gazebo_data", m_using_gazebo_data_param);
    m_nh.getParam(nd_name + "/noise_dist_th_m", m_noise_dist_th_m_param);
    m_nh.getParam(nd_name + "/noise_cnt_th", m_noise_cnt_th_param);
    m_nh.getParam(nd_name + "/process_freq", m_process_freq_param);

    if (m_aruco_parser_param == "missing") { ROS_ERROR_STREAM("[aruco_tracking] m_aruco_parser_param is missing"); return false; }
    else if (__isnan(m_big_marker_id_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_big_marker_id_param is NAN"); return false; }
    else if (__isnan(m_small_marker_id_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_small_marker_id_param is NAN"); return false; }
    else if (__isnan(m_big_marker_size_m_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_big_marker_size_m_param is NAN"); return false; }
    else if (__isnan(m_small_marker_size_m_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_small_marker_size_m_param is NAN"); return false; }
    else if (__isnan(m_filter_buf_size_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_filter_buf_size_param is NAN"); return false; }
    else if (__isnan(m_estimating_method_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_estimating_method_param is NAN"); return false; }
    else if (__isnan(m_noise_dist_th_m_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_noise_dist_th_m_param is NAN"); return false; }
    else if (__isnan(m_noise_cnt_th_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_noise_cnt_th_param is NAN"); return false; }
    else if (__isnan(m_process_freq_param)) { ROS_ERROR_STREAM("[aruco_tracking] m_process_freq_param is NAN"); return false; }

    return true;
}

bool ArucoTracking::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();
    string ns_name = ros::this_node::getNamespace();
    string image_topic_name;
    if (m_using_gazebo_data_param) image_topic_name = "/camera/rgb/image_raw";
    else image_topic_name = ns_name + "/usb_cam/image_raw";

    // Initialize subscriber
    m_image_sub = m_nh.subscribe<sensor_msgs::Image>(image_topic_name, 1, boost::bind(&ArucoTracking::ImageCallback, this, _1));
    
    // Initialize publisher
    m_image_pub = m_nh.advertise<sensor_msgs::Image>(nd_name + "/output_video", 10);
    m_tf_list_pub = m_nh.advertise<tf2_msgs::TFMessage>(nd_name + "/tf_list", 10);
    m_markers_pub = m_nh.advertise<visualization_msgs::MarkerArray> (nd_name + "/aruco_markers", 1);
    m_target_state_pub = m_nh.advertise<kuam_msgs::MarkerState> (nd_name + "/target_state", 1);
    m_target_states_pub = m_nh.advertise<kuam_msgs::MarkerStateArray> (nd_name + "/target_states", 1);
    m_target_list_pub = m_nh.advertise<geometry_msgs::PoseArray> (nd_name + "/target_list", 1);
    
    // Initialize timer
    m_image_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &ArucoTracking::ProcessTimerCallback, this);

    return true;
}

bool ArucoTracking::InitMarkers()
{
    //// target markers
    std_msgs::ColorRGBA red;        red.r = 1.0f;       red.g = 0.0f;       red.b = 0.0f;
    std_msgs::ColorRGBA green;      green.r = 0.0f;     green.g = 1.0f;     green.b = 0.0f;
    std_msgs::ColorRGBA blue;       blue.r = 0.0f;      blue.g = 0.0f;      blue.b = 1.0f;
    std_msgs::ColorRGBA yellow;     yellow.r = 1.0f;    yellow.g = 1.0f;    yellow.b = 0.0f;
    std_msgs::ColorRGBA white;      white.r = 1.0f;     white.g = 1.0f;     white.b = 1.0f;

    for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
        MetaMarkers target_marker;
        // Without filter. green
        target_marker.trajectory.ns = "target/trajectory";
        target_marker.trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        target_marker.trajectory.action = visualization_msgs::Marker::ADD;
        target_marker.trajectory.scale.x = 0.05;
        target_marker.trajectory.pose.orientation.w = 1.0;
        target_marker.trajectory.pose.orientation.x = 0.0;
        target_marker.trajectory.pose.orientation.y = 0.0;
        target_marker.trajectory.pose.orientation.z = 0.0;
        target_marker.trajectory.lifetime = ros::Duration(0.3);
        target_marker.is_trajectory_add = false;

        target_marker.current_point.ns = "target/current_point";
        target_marker.current_point.type = visualization_msgs::Marker::SPHERE;
        target_marker.current_point.action = visualization_msgs::Marker::ADD;
        target_marker.current_point.scale.x = 0.15;
        target_marker.current_point.scale.y = 0.15;
        target_marker.current_point.scale.z = 0.15;
        target_marker.current_point.pose.orientation.w = 1.0;
        target_marker.current_point.pose.orientation.x = 0.0;
        target_marker.current_point.pose.orientation.y = 0.0;
        target_marker.current_point.pose.orientation.z = 0.0;
        target_marker.current_point.lifetime = ros::Duration(0.3);
        target_marker.is_current_point_add = false;
        
        target_marker.txt.ns = "target/txt";
        target_marker.txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        target_marker.txt.action = visualization_msgs::Marker::ADD;
        target_marker.txt.scale.z = 0.1;
        target_marker.txt.pose.orientation.w = 1.0;
        target_marker.txt.pose.orientation.x = 0.0;
        target_marker.txt.pose.orientation.y = 0.0;
        target_marker.txt.pose.orientation.z = 0.0;
        target_marker.txt.lifetime = ros::Duration(0.3);
        target_marker.is_txt_add = false;

        switch (method){
            case (int)EstimatingMethod::WOF:
                green.a = 0.4f;
                target_marker.trajectory.id = (int)EstimatingMethod::WOF;
                target_marker.trajectory.color = green;

                green.a = 1.0f;
                target_marker.current_point.id = (int)EstimatingMethod::WOF;
                target_marker.current_point.color = green;
                
                white.a = 1.0f;
                target_marker.txt.id = (int)EstimatingMethod::WOF;
                target_marker.txt.color = white;

                break;
            case (int)EstimatingMethod::MAF:
                red.a = 0.4f;
                target_marker.trajectory.id = (int)EstimatingMethod::MAF;
                target_marker.trajectory.color = red;

                red.a = 1.0f;
                target_marker.current_point.id = (int)EstimatingMethod::MAF;
                target_marker.current_point.color = red;
                
                white.a = 1.0f;
                target_marker.txt.id = (int)EstimatingMethod::MAF;
                target_marker.txt.color = white;

                break;
            case (int)EstimatingMethod::EMAF:
                yellow.a = 0.4f;
                target_marker.trajectory.id = (int)EstimatingMethod::EMAF;
                target_marker.trajectory.color = yellow;

                yellow.a = 1.0f;
                target_marker.current_point.id = (int)EstimatingMethod::EMAF;
                target_marker.current_point.color = yellow;
                
                white.a = 1.0f;
                target_marker.txt.id = (int)EstimatingMethod::EMAF;
                target_marker.txt.color = white;

                break;
            default:
                break;
        }
        m_target_markers.push_back(target_marker);
    }
}

void ArucoTracking::ProcessTimerCallback(const ros::TimerEvent& event)
{
    if (m_cv_ptr != nullptr){
        vector<int> ids;
        bool is_detected = false;
        geometry_msgs::Pose target_pose;
        MarkerPoseEstimating(ids, target_pose, is_detected);

        TargetStateEstimating(ids, target_pose, is_detected);
        VisualizeTarget();
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
    for (auto id : ids){
        if (m_id_to_markersize_map.find(id) != m_id_to_markersize_map.end()){
            if (IsNoise()) is_detected = false;
            else{
                is_detected = true;
                m_target.id = id;
                m_target.marker_size_m = m_id_to_markersize_map.find(id)->second;
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

            if(m_show_rejected && rejected.size() > 0)
                aruco::drawDetectedMarkers(copy_image, rejected, noArray(), Scalar(100, 0, 255));
        }
    }

    sensor_msgs::Image img_msg; // >> message to be sent
    m_cv_ptr->image = copy_image;
    m_cv_ptr->header.frame_id = CAMERA_FRAME;
    img_msg = *m_cv_ptr->toImageMsg();
    m_image_pub.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);    

    return true;
}

bool ArucoTracking::Camera2World(const vector<vector<Point2f>> corners, const vector<int> ids, const vector<Vec3d> rvecs, const vector<Vec3d> tvecs, geometry_msgs::Pose& target_pose)
{
    // Create and publish tf message for each marker
    tf2_msgs::TFMessage tf_msg_list;
    for (auto i = 0; i < rvecs.size(); ++i){
        if (ids[i] == m_target.id){
            // ROS_ERROR("%f, %f", corners[i][0].x, corners[i][0].y);
            // ROS_ERROR("%f, %f", corners[i][1].x, corners[i][1].y);
            // ROS_ERROR("%f, %f", corners[i][2].x, corners[i][2].y);
            // ROS_ERROR("%f, %f", corners[i][3].x, corners[i][3].y);

            geometry_msgs::TransformStamped aruco_tf_stamped;
            aruco_tf_stamped.header.stamp = ros::Time::now();
            aruco_tf_stamped.header.frame_id = CAMERA_FRAME;

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

            // Marker corner coordinate 
            // geometry_msgs::TransformStamped transformStamped;
            // geometry_msgs::Pose transformed_pose;
            // try{
            //     transformStamped = m_tfBuffer.lookupTransform("base_link", "map", ros::Time(0));
            //     tf2::doTransform(target_state_ptr->pose, transformed_pose, transformStamped);
            // }
            // catch (tf2::TransformException &ex) {
            //     ROS_WARN("%s", ex.what());
            //     ros::Duration(1.0).sleep();
            // }

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
    kuam_msgs::MarkerState target_msg;
    target_msg.header.frame_id = CAMERA_FRAME;
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

    // kuam_msgs::MarkerStateArray target_states_msg;
    // target_states_msg.states.push_back(target_wof_msg);
    // target_states_msg.states.push_back(target_maf_msg);
    // // target_states_msg.states.push_back(target_emaf_msg);
    // m_target_states_pub.publish(target_states_msg);
}

void ArucoTracking::VisualizeTarget()
{
    geometry_msgs::Point pos;
    geometry_msgs::Point origin;

    if (m_target.is_detected){
        for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
            if (m_compare_mode_param || (method == m_estimating_method_param)){
                // current_point
                pos.x = m_target.state[method].position.x();
                pos.y = m_target.state[method].position.y();
                pos.z = m_target.state[method].position.z();

                m_target_markers[method].current_point.header.frame_id = CAMERA_FRAME;
                m_target_markers[method].current_point.header.stamp = ros::Time(0);

                m_target_markers[method].current_point.pose.position = pos;
                m_target_markers[method].is_current_point_add = true;

                // txt
                m_target_markers[method].txt.header.frame_id = CAMERA_FRAME;;
                m_target_markers[method].txt.header.stamp = ros::Time(0);

                m_target_markers[method].txt.pose.position = pos;
                m_target_markers[method].txt.pose.position.z += 0.2;
                m_target_markers[method].txt.text = "dist: " + m_utils.ToString(m_utils.Distance3D(origin, pos)) + " [m]";
                m_target_markers[method].is_txt_add = true;
            }
        }
    }

    for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
        m_target_markers[method].self.markers.clear();
        if (m_target_markers[method].is_trajectory_add) m_target_markers[method].self.markers.push_back(m_target_markers[method].trajectory);
        if (m_target_markers[method].is_current_point_add) m_target_markers[method].self.markers.push_back(m_target_markers[method].current_point);
        if (m_target_markers[method].is_txt_add) m_target_markers[method].self.markers.push_back(m_target_markers[method].txt);
        m_target_markers[method].is_trajectory_add = m_target_markers[method].is_current_point_add = m_target_markers[method].is_txt_add = false;
    }
    
    visualization_msgs::MarkerArray visualization_markers;
    for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
        visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                            m_target_markers[method].self.markers.begin(), m_target_markers[method].self.markers.end());
    }

    m_markers_pub.publish(visualization_markers);
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
