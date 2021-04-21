#ifndef __ARUCO_TRACKING_H__
#define __ARUCO_TRACKING_H__

#include <ros/ros.h>

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
#include <string>

#include <aruco_tracking/parser.h>

namespace kuam{

class ArucoTracking
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    kuam::Parser m_parser;

public:
    ArucoTracking();
    virtual ~ArucoTracking();

private:
    // Subscriber
    ros::Subscriber m_image_sub;

    // Publisher
    ros::Publisher m_image_pub;
    
    // Timer
    ros::Timer m_timer;

    // Param
    std::string m_aruco_parser_param; // Dictionary
    int m_dictionary_id_param;
    // std::string m_input_video_param;
    // int m_camera_id_param;
    std::string m_instrinsic_param;
    float m_marker_length_param;
    std::string m_marker_detector_param_param;
    bool m_show_rejected_candidates_param;
    int m_coner_refinement_param;

    const std::string OPENCV_WINDOW;
    bool m_do_estimate_pose;
    bool m_show_rejected;
    float m_marker_length;
    cv::Ptr<cv::aruco::DetectorParameters> m_detector_params;
    cv::Ptr<cv::aruco::Dictionary> m_dictionary;
    cv::Mat m_cam_matrix;
    cv::Mat m_dist_coeffs;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    
    // void ImageCallback(const ros::TimerEvent& event);
    void ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr);

    bool DetectingMarker(cv::Mat input_image);
    bool PoseEstimatingMarker(cv::Mat input_image);

    void Publish();
};
}
#endif //  __ARUCO_TRACKING_H__