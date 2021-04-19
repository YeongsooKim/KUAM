#include "aruco_tracking/aruco_tracking.h"

namespace kuam{

ArucoTracking::ArucoTracking() :
    OPENCV_WINDOW("Image window")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();

    cv::namedWindow(OPENCV_WINDOW);
}
ArucoTracking::~ArucoTracking()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

bool ArucoTracking::GetParam()
{

    return true;
}

bool ArucoTracking::InitFlag()
{

    return true;
}

bool ArucoTracking::InitROS()
{
    // package, node, topic name
    std::string node_name_with_namespace = ros::this_node::getName();

    // Initialize subscriber
    m_image_sub = m_nh.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 1, boost::bind(&ArucoTracking::ImageCallback, this, _1));
    
    // Initialize publisher
    m_image_pub = m_nh.advertise<sensor_msgs::Image>(node_name_with_namespace + "/output_video", 10);

    return true;
}

void ArucoTracking::ImageCallback(const sensor_msgs::Image::ConstPtr &img_ptr)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    // if (cv::waitKey(10)==27) break;

    
    m_image_pub.publish(cv_ptr->toImageMsg());

}


void ArucoTracking::Publish()
{}

}