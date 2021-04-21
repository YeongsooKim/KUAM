#include "aruco_tracking/aruco_tracking.h"

#include <opencv2/core/persistence.hpp>

using namespace std;
using namespace cv;

namespace kuam{

ArucoTracking::ArucoTracking() :
    OPENCV_WINDOW("Image window"),
    m_aruco_parser_param("missing")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();

    namedWindow(OPENCV_WINDOW);
    m_parser.ReadFile(m_aruco_parser_param, m_do_estimate_pose, m_show_rejected, 
                        m_marker_length, m_detector_params, m_dictionary, m_cam_matrix, m_dist_coeffs);
}
ArucoTracking::~ArucoTracking()
{
    destroyWindow(OPENCV_WINDOW);
}

bool ArucoTracking::GetParam()
{
    string node_name_with_namespace = ros::this_node::getName();

    m_nh.getParam(node_name_with_namespace + "/arcuo_parser", m_aruco_parser_param);
    m_nh.getParam(node_name_with_namespace + "/dictionary_id", m_dictionary_id_param);
    // m_nh.getParam(node_name_with_namespace + "/input_video", m_input_video_param);
    // m_nh.getParam(node_name_with_namespace + "/camera_id", m_camera_id_param);
    m_nh.getParam(node_name_with_namespace + "/instrinsic", m_instrinsic_param);
    m_nh.getParam(node_name_with_namespace + "/marker_length", m_marker_length_param);
    m_nh.getParam(node_name_with_namespace + "/marker_detector_param", m_marker_detector_param_param);
    m_nh.getParam(node_name_with_namespace + "/show_rejected_candidates", m_show_rejected_candidates_param);
    m_nh.getParam(node_name_with_namespace + "/coner_refinement", m_coner_refinement_param);

    if (m_aruco_parser_param == "missing") { ROS_ERROR_STREAM("m_aruco_parser_param is missing"); return false; }

    return true;
}

bool ArucoTracking::InitFlag()
{

    return true;
}

bool ArucoTracking::InitROS()
{
    // package, node, topic name
    string node_name_with_namespace = ros::this_node::getName();

    // Initialize subscriber
    m_image_sub = m_nh.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 1, boost::bind(&ArucoTracking::ImageCallback, this, _1));
    
    // Initialize publisher
    m_image_pub = m_nh.advertise<sensor_msgs::Image>(node_name_with_namespace + "/output_video", 10);

    return true;
}


// bool ArucoTracking::InitParam()
// {
//     m_detector_params = aruco::DetectorParameters::create();
    
//     bool readOk = readDetectorParameters("/home/ys/kuam_ws/src/KUAM/config/detector_params.yml", m_detector_params);
//     // bool readOk = readDetectorParameters(m_marker_detector_param_param, m_detector_params);
//     if(!readOk) {
//         ROS_ERROR_STREAM("Invalid detector parameters file");
//         return false;
//     }

//     //override cornerRefinementMethod read from config file
//     // m_detector_params->cornerRefinementMethod = m_coner_refinement_param;
//     // ROS_INFO("Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): %d", m_coner_refinement_param);

//     // int camId = m_camera_id_param;

//     Ptr<aruco::Dictionary> m_dictionary =
//         aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(m_dictionary_id_param));

//     readOk = readCameraParameters(m_instrinsic_param, m_camera_matrix, m_dist_coeffs);
//     if(!readOk) {
//         ROS_ERROR_STREAM("Invalid camera file");
//         return false;
//     }
// }

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
    
    // DetectingMarker(cv_ptr->image);
    PoseEstimatingMarker(cv_ptr->image);
    waitKey(3);
    
    m_image_pub.publish(cv_ptr->toImageMsg());
}


bool ArucoTracking::DetectingMarker(Mat input_image)
{

    Mat output_image;
    input_image.copyTo(output_image);

    vector<int> markerIds;
    vector<vector<Point2f> > markerCorners, rejectedCandidates;
    Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
    Ptr<aruco::Dictionary> m_dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    aruco::detectMarkers(input_image, m_dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    // if at least one marker detected
    if (markerIds.size() > 0){
        aruco::drawDetectedMarkers(output_image, markerCorners, markerIds);
    }
        
    imshow(OPENCV_WINDOW, output_image);

    return true;
}

bool ArucoTracking::PoseEstimatingMarker(Mat input_image)
{
    static int waitTime = 0;
    static double totalTime = 0;
    static int totalIterations = 0;

    Mat copy_image;

    double tick = (double)getTickCount();

    vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    vector< Vec3d > rvecs, tvecs;

    // detect markers and estimate pose
    aruco::detectMarkers(input_image, m_dictionary, corners, ids, m_detector_params, rejected);
    if(m_do_estimate_pose && ids.size() > 0){
        aruco::estimatePoseSingleMarkers(corners, m_marker_length, m_cam_matrix, m_dist_coeffs, rvecs,tvecs);
    }

    double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
    totalTime += currentTime;
    totalIterations++;
    if(totalIterations % 30 == 0) {
        ROS_INFO("Detection Time = %f ms (Mean = %f ms)", currentTime * 1000, 1000 * totalTime / double(totalIterations));
    }

    // draw results
    input_image.copyTo(copy_image);
    if(ids.size() > 0) {
        aruco::drawDetectedMarkers(copy_image, corners, ids);

        if(m_do_estimate_pose) {
            for(unsigned int i = 0; i < ids.size(); i++)
                aruco::drawAxis(copy_image, m_cam_matrix, m_dist_coeffs, rvecs[i], tvecs[i],
                                m_marker_length * 0.5f);
        }
    }

    if(m_show_rejected && rejected.size() > 0)
        aruco::drawDetectedMarkers(copy_image, rejected, noArray(), Scalar(100, 0, 255));

    imshow(OPENCV_WINDOW, copy_image);
}

void ArucoTracking::Publish()
{}

}