#ifndef __ARUCO_TRACKING_H__
#define __ARUCO_TRACKING_H__

#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <map>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio_c.h>
#include "opencv2/aruco/dictionary.hpp"

#include <kuam_aruco_tracking/utils/util_marker.h>
#include <kuam_aruco_tracking/utils/util_setpoint.h>
#include <kuam_aruco_tracking/parser.h>
#include <kuam_aruco_tracking/target.h>

#include <tf2_msgs/TFMessage.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include <kuam_msgs/ArucoState.h>

using namespace std;
using namespace cv;

namespace kuam{

class ArucoTracking
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    ros::NodeHandle m_p_nh;

    UtilMarker m_util_marker;
    UtilSetpoint m_util_setpoint;
    Parser m_parser;
    vector<Target> m_targets;
    cv_bridge::CvImagePtr m_cv_ptr;

public:
    ArucoTracking();
    virtual ~ArucoTracking();

private:
    // Subscriber
    ros::Subscriber m_image_sub;

    // Publisher
    ros::Publisher m_image_pub;
    ros::Publisher m_tf_list_pub;
    ros::Publisher m_visual_pub;
    ros::Publisher m_target_state_pub;

    // Timer
    ros::Timer m_image_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;
    string m_calib_path_param;
    string m_detector_params_path_param;
    string m_camera_frame_id_param;
    bool m_is_simulation_param;
    int m_dictionaryID_param;
    int m_marker_cnt_th_param;
    float m_big_marker_trans_param;
    float m_medium_marker_trans_param;
    float m_small_marker_trans_param;

    vector<int> m_big_marker_ids_param;
    vector<int> m_medium_marker_ids_param;
    vector<int> m_small_marker_ids_param;
    float m_big_marker_size_m_param;
    float m_medium_marker_size_m_param;
    float m_small_marker_size_m_param;
    int m_filter_buf_size_param;
    int m_estimating_method_param;
    float m_noise_dist_th_m_param;
    float m_noise_cnt_th_param;
    bool m_compare_mode_param;

    // Const value
    const int MARKER_ID_QUEUE_SIZE;

    // ArUco variable
    Ptr<aruco::DetectorParameters> m_detector_params;
    Ptr<aruco::Dictionary> m_dictionary;
    Mat m_cam_matrix;
    Mat m_dist_coeffs;
    vector<bool> m_is_small_id_queue;
    bool m_fix_small_marker;
    ros::Time m_last_enough_time;
    vector<int> m_marker_ids;
    using int2pose = map < int, geometry_msgs::Pose >;

    // Time variable
    ros::Time m_last_detected_time;

    // Video variable
    VideoCapture m_input_video;
    const string OPENCV_WINDOW;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitMarker();
    
    void ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr);
    void ProcessTimerCallback(const ros::TimerEvent& event);

    void SelectMarkers(vector<vector<Point2f>>& corners, vector<int>& ids,
                    vector<vector<Point2f>>& s_corners, vector<int>& s_ids,
                    vector<vector<Point2f>>& m_corners, vector<int>& m_ids,
                    vector<vector<Point2f>>& b_corners, vector<int>& b_ids);
    void NoiseFilter(vector<vector<Point2f>>& s_corners, vector<int>& s_ids,
                    vector<vector<Point2f>>& m_corners, vector<int>& m_ids,
                    vector<vector<Point2f>>& b_corners, vector<int>& b_ids);
    bool Camera2World(const vector<vector<Point2f>> corners, const vector<int> ids, const vector<Vec3d> rvecs, 
        const vector<Vec3d> tvecs, int2pose& int_to_pose, tf2_msgs::TFMessage& tf_msg_list);
    void MarkerUpdate(const vector<int> b_ids, const vector<int> m_ids, const vector<int> s_ids, int2pose int_to_pose);
    void ImagePub(Mat image, const vector<vector<Point2f>> s_corners, const vector<int> s_ids,
                            const vector<vector<Point2f>> m_corners, const vector<int> m_ids,
                            const vector<vector<Point2f>> b_corners, const vector<int> b_ids);
    void TargetPub();
};
}
#endif //  __ARUCO_TRACKING_H__