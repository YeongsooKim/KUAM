#ifndef __ARUCO_TRACKING_H__
#define __ARUCO_TRACKING_H__

#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
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
#include <kuam_aruco_tracking/frequency_degree.h>

#include <tf2_msgs/TFMessage.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include <kuam_msgs/ArucoState.h>
#include <kuam_msgs/ArucoStates.h>
#include <kuam_msgs/ArucoVisuals.h>

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
    map<int, FrequencyDegree> m_id2frequencyDegrees;
    map<int, Z2NormalAngle> m_id2ZbtNormalAngles;
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
    double m_plane_threshold_param;
    int m_plane_iterations_param;
    double m_angle_deg_threshold_param;

    int m_filter_buf_size_param;
    int m_estimating_method_param;
    float m_noise_dist_th_m_param;
    float m_noise_cnt_th_param;
    bool m_compare_mode_param;

    int m_marker_size_type_num_param;
    float m_big_marker_trans_param;
    float m_medium_marker_trans_param;
    float m_small_marker_trans_param;

    int m_dft_buf_size_param;
    int m_freq_deg_buf_size_param;
    int m_dft_integral_start_point_param;
    double m_freq_degree_th_param;
    double m_diff_th_m_param;
    
    // Const value
    const int MARKER_ID_QUEUE_SIZE;

    // ArUco variable
    Ptr<aruco::DetectorParameters> m_detector_params;
    Ptr<aruco::Dictionary> m_dictionary;
    Mat m_cam_matrix;
    Mat m_dist_coeffs;
    vector<int> m_marker_ids;
    vector<vector<int>> m_marker_ids_vec;
    vector<double> m_marker_sizes_m;
    using int2pose = map < int, geometry_msgs::Pose >;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitMarker();
    
    void ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr);
    void ProcessTimerCallback(const ros::TimerEvent& event);

    void SelectMarker(const vector<int> target_ids, vector<vector<Point2f>>& corners, vector<int>& ids,
                    vector<vector<Point2f>>& discrete_corners, vector<int>& discrete_ids);
    void NoiseFilter(vector < vector<vector<Point2f>> >& corners, vector < vector<int> >& ids);
    bool GetTransformation(const vector<vector<Point2f>> corners, const vector<int> ids, const vector<Vec3d> rvecs, 
        const vector<Vec3d> tvecs, int2pose& int_to_pose, tf2_msgs::TFMessage& tf_msg_list);
    void TargetStateEstimating(const vector < vector<int> > ids_vec, int2pose int_to_pose);
    void CalFrequencyDegree();
    void SetArucoMessages(kuam_msgs::ArucoStates& ac_states_msg, kuam_msgs::ArucoVisuals& ac_visuals_msg);
    void ImagePub(Mat image, const vector < vector<vector<Point2f>> > corners_vec, const vector < vector<int> > ids_vec);
    void TargetPub(kuam_msgs::ArucoStates ac_states_msg, kuam_msgs::ArucoVisuals ac_visuals_msg);
};
}
#endif //  __ARUCO_TRACKING_H__