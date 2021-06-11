#ifndef __ARUCO_TRACKING_H__
#define __ARUCO_TRACKING_H__

#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>

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

#include <kuam_aruco_tracking/parser.h>
#include <kuam_aruco_tracking/utils.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include <kuam_msgs/ArucoState.h>

using namespace std;
using namespace cv;

namespace kuam{

// Target without filter
struct State{
    Eigen::Vector3d position;
    Eigen::Vector3d prev_position;
    vector < vector < Point2f > > corners;
};

using id2markersizes = map < int , float >;

struct Target{
    int id;
    float marker_size_m;
    bool is_init;

    bool is_detected;
    ros::Time last_detected_time;

    vector<State> state;
};

enum class EstimatingMethod : int
{
    WOF,    // Without filter
    MAF,    // Moving average filter
    EMAF,   // Exponential moving average filter
    // KF,     // Kalman filter

    ItemNum
};

class ArucoTracking
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    Parser m_parser;
    Utils m_utils;
    Target m_target;
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
    ros::Publisher m_target_list_pub;
    ros::Publisher m_cnt_pub;
    ros::Publisher m_target_marker_pub;

    // Timer
    ros::Timer m_image_timer;

    // Param
    bool m_is_eval_param;
    string m_aruco_parser_param; // Dictionary
    int m_big_marker_id_param;
    int m_small_marker_id_param;
    float m_big_marker_size_m_param;
    float m_small_marker_size_m_param;
    int m_filter_buf_size_param;
    int m_estimating_method_param;
    bool m_compare_mode_param;
    bool m_using_gazebo_data_param;
    float m_noise_dist_th_m_param;
    float m_noise_cnt_th_param;
    float m_process_freq_param;
    int m_marker_cnt_th_param;

    // Const value
    const int MARKER_ID_STACK_SIZE;

    // ArUco variable
    Ptr<aruco::DetectorParameters> m_detector_params;
    Ptr<aruco::Dictionary> m_dictionary;
    Mat m_cam_matrix;
    Mat m_dist_coeffs;
    bool m_do_estimate_pose;
    bool m_show_rejected;
    id2markersizes m_id_to_markersize_map;
    vector<int> m_detected_marker_num_stack;
    bool m_fix_small_marker;
    ros::Time m_last_enough_time;

    // Time variable
    ros::Time m_last_detected_time;
    ros::Time m_last_noise_check_time;

    // Video variable
    VideoCapture m_input_video;
    const string OPENCV_WINDOW;


    geometry_msgs::PoseArray m_target_pose_list;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    
    void ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr);
    void ProcessTimerCallback(const ros::TimerEvent& event);

    bool Convert2CVImg(const sensor_msgs::Image::ConstPtr &img_ptr);
    bool MarkerPoseEstimating(vector<int>& ids, geometry_msgs::Pose& target_pose, bool& is_detected);
    bool TargetStateEstimating(const vector<int> ids, const geometry_msgs::Pose pose, const bool is_detected);

    bool WithoutFilter(const Eigen::Vector3d pos, State& state);
    bool MovingAvgFilter(const Eigen::Vector3d pos, State& state);
    bool ExpMovingAvgFilter(const Eigen::Vector3d pos, State& state);
    
    tf2::Vector3 CvVector3d2TfVector3(const Vec3d &vec);
    tf2::Quaternion CvVector3d2TfQuarternion(const Vec3d &rotation_vector);
    tf2::Transform CreateTransform(const Vec3d &tvec, const Vec3d &rotation_vector);

    bool Camera2World(const vector<vector<Point2f>> corners, const vector<int> ids, const vector<Vec3d> rvecs, const vector<Vec3d> tvecs, geometry_msgs::Pose& target_pose);
    bool IsNoise();
    bool IsNoise(const geometry_msgs::Pose target_pose);
};
}
#endif //  __ARUCO_TRACKING_H__