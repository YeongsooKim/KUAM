#ifndef __ARUCO_TRACKING_H__
#define __ARUCO_TRACKING_H__

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio_c.h>

namespace kuam{

class ArucoTracking
{
private:
    // Node Handler
	ros::NodeHandle m_nh;

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

    // Flag
    const std::string OPENCV_WINDOW;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    
    // void ImageCallback(const ros::TimerEvent& event);
    void ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr);

    void Publish();
};
}
#endif //  __ARUCO_TRACKING_H__