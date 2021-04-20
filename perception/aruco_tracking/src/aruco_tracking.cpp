#include "aruco_tracking/aruco_tracking.h"

#include <opencv2/aruco.hpp>
#include "opencv2/aruco/dictionary.hpp"

namespace kuam{

ArucoTracking::ArucoTracking() :
    OPENCV_WINDOW("Image window"),
    m_dictionary_id_param(NAN),
    m_marker_length_param(NAN)
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
    std::string node_name_with_namespace = ros::this_node::getName();

    m_nh.getParam(node_name_with_namespace + "/dictionary_id", m_dictionary_id_param);
    m_nh.getParam(node_name_with_namespace + "/marker_length", m_marker_length_param);

    if (__isnan(m_dictionary_id_param)) { ROS_ERROR_STREAM("m_dictionary_id_param is NAN"); return false; }
    if (__isnan(m_marker_length_param)) { ROS_ERROR_STREAM("m_marker_length_param is NAN"); return false; }

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
    
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    DetectingMarker(cv_ptr->image);
    cv::waitKey(3);
    // if (cv::waitKey(10)==27) break;

    
    m_image_pub.publish(cv_ptr->toImageMsg());

}


bool ArucoTracking::DetectingMarker(cv::Mat input_image)
{

    cv::Mat output_image;
    input_image.copyTo(output_image);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(input_image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    // if at least one marker detected
    if (markerIds.size() > 0){
        cv::aruco::drawDetectedMarkers(output_image, markerCorners, markerIds);
    }
        
    cv::imshow(OPENCV_WINDOW, output_image);

    return true;
}

void ArucoTracking::Publish()
{}

}