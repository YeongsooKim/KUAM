#ifndef __ARUCO_TRACKING_H__
#define __ARUCO_TRACKING_H__

#include <ros/ros.h>
#include <string>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <alfons_msgs/ArucoInfo.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio_c.h>
#include "opencv2/aruco/dictionary.hpp"

#include <aruco_tracking/parser.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

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
    ros::Publisher m_aruco_info_pub;
    ros::Publisher m_tf_list_pub;
    
    // Timer
    ros::Timer m_timer;

    // Param
    std::string m_aruco_parser_param; // Dictionary

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

    bool MarkerDetecting(Mat input_image, Mat& output_image);
    bool MarkerPoseEstimating(Mat input_image, Mat& output_image);

    tf2::Vector3 CvVector3d2TfVector3(const Vec3d &vec);
    tf2::Quaternion CvVector3d2TfQuarternion(const Vec3d &rotation_vector);
    tf2::Transform CreateTransform(const Vec3d &tvec, const Vec3d &rotation_vector);

    void Publish();
};
}
#endif //  __ARUCO_TRACKING_H__