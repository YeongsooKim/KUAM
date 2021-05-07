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
#include <geometry_msgs/Twist.h>

#include <visualization_msgs/MarkerArray.h>

#include <kuam_msgs/TargetState.h>
#include <kuam_msgs/TargetStateArray.h>

using namespace std;
using namespace cv;

namespace kuam{

// Target without filter
struct State{
    Eigen::Vector3d position;
    Eigen::Vector3d prev_position;
};

struct Target{
    int id;
    bool is_init;

    bool is_detected;
    ros::Time last_detected_time;

    vector<State> state;
};

struct MetaMarkers{
    visualization_msgs::MarkerArray self;
    visualization_msgs::Marker trajectory;
    visualization_msgs::Marker current_point;
    visualization_msgs::Marker txt;

    bool is_trajectory_add;
    bool is_current_point_add;
    bool is_txt_add;
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

    vector<MetaMarkers> m_target_markers; // target markers

public:
    ArucoTracking();
    virtual ~ArucoTracking();

private:
    // Subscriber
    ros::Subscriber m_image_sub;

    // Publisher
    ros::Publisher m_image_pub;
    ros::Publisher m_tf_list_pub;
    ros::Publisher m_markers_pub;
    ros::Publisher m_target_state_pub;
    ros::Publisher m_target_states_pub;
    
    // Timer
    ros::Timer m_image_timer;

    // Param
    string m_aruco_parser_param; // Dictionary
    int m_target_marker_id_param; // 
    int m_filter_buf_size_param; // 
    int m_estimating_method_param;
    bool m_compare_mode_param;
    bool m_using_image_msg_param;

    VideoCapture m_input_video;
    const string OPENCV_WINDOW;
    bool m_do_estimate_pose;
    bool m_show_rejected;
    float m_marker_length;
    Ptr<aruco::DetectorParameters> m_detector_params;
    Ptr<aruco::Dictionary> m_dictionary;
    Mat m_cam_matrix;
    Mat m_dist_coeffs;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitMarkers();
    
    void ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr);
    void TestTimerCallback(const ros::TimerEvent& event);
    void ImageTimerCallback(const ros::TimerEvent& event);

    bool Convert2CVImg(const sensor_msgs::Image::ConstPtr &img_ptr);
    bool MarkerPoseEstimating(vector<int>& ids, vector<Vec3d>& rvecs, vector<Vec3d>& tvecs);
    bool Camera2World(const vector<int> ids, const vector<Vec3d> rvecs, const vector<Vec3d> tvecs, geometry_msgs::Pose& target_pose, bool& has_target);
    bool TargetStateEstimating(const geometry_msgs::Pose pose, bool& has_target);
    void VisualizeTarget();
    void Publish();

    bool WithoutFilter(const Eigen::Vector3d pos, State& state);
    bool MovingAvgFilter(const Eigen::Vector3d pos, State& state);
    bool ExpMovingAvgFilter(const Eigen::Vector3d pos, State& state);
    
    tf2::Vector3 CvVector3d2TfVector3(const Vec3d &vec);
    tf2::Quaternion CvVector3d2TfQuarternion(const Vec3d &rotation_vector);
    tf2::Transform CreateTransform(const Vec3d &tvec, const Vec3d &rotation_vector);
};
}
#endif //  __ARUCO_TRACKING_H__