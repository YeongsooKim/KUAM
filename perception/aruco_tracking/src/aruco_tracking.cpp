#include "aruco_tracking/aruco_tracking.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

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
    m_aruco_info_pub = m_nh.advertise<alfons_msgs::ArucoInfo>(node_name_with_namespace + "/aruco_info", 10);
    m_tf_list_pub = m_nh.advertise<tf2_msgs::TFMessage>(node_name_with_namespace + "/tf_list", 10);

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
    
    Mat output_image;
    // MarkerDetecting(cv_ptr->image, output_image);
    MarkerPoseEstimating(cv_ptr->image, output_image);
    cv_ptr->image = output_image;
    
    // imshow(OPENCV_WINDOW, output_image);
    // waitKey(1);
    
    m_image_pub.publish(cv_ptr->toImageMsg());
}

bool ArucoTracking::MarkerDetecting(Mat input_image, Mat& output_image)
{
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
        

    return true;
}

bool ArucoTracking::MarkerPoseEstimating(Mat input_image, Mat& output_image)
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

    // publish aruco info:
    alfons_msgs::ArucoInfo ar_msg;
    ar_msg.header.stamp = ros::Time::now();
    ar_msg.header.frame_id = "camera";
    for(int i = 0;i<ids.size();i++){
        vector<Point2f> one_corner = corners[i];
        ar_msg.marker_ids.push_back(ids[i]);
        ar_msg.center_x_px.push_back((one_corner[0].x+one_corner[1].x+one_corner[2].x+one_corner[3].x)/4);
        ar_msg.center_y_px.push_back((one_corner[0].y+one_corner[1].y+one_corner[2].y+one_corner[3].y)/4);
    }
    m_aruco_info_pub.publish(ar_msg);
 
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


    // Publish TFs for each of the markers
    static tf2_ros::TransformBroadcaster aruco_tf_broadcaster;
    auto stamp = ros::Time::now();

    // Create and publish tf message for each marker
    tf2_msgs::TFMessage tf_msg_list;
    for (auto i = 0; i < rvecs.size(); ++i){
        geometry_msgs::TransformStamped aruco_tf_stamped;
        aruco_tf_stamped.header.stamp = stamp;
        aruco_tf_stamped.header.frame_id = "camera";

        auto translation_vector = tvecs[i];
        auto rotation_vector = rvecs[i];
        auto transform = CreateTransform(translation_vector, rotation_vector);
        stringstream ss;
        string marker_tf_prefix = "marker";
        ss << marker_tf_prefix << ids[i];
        aruco_tf_stamped.child_frame_id = ss.str();
        aruco_tf_stamped.transform.translation.x = transform.getOrigin().getX();
        aruco_tf_stamped.transform.translation.y = transform.getOrigin().getY();
        aruco_tf_stamped.transform.translation.z = transform.getOrigin().getZ();
        aruco_tf_stamped.transform.rotation.x = transform.getRotation().getX();
        aruco_tf_stamped.transform.rotation.y = transform.getRotation().getY();
        aruco_tf_stamped.transform.rotation.z = transform.getRotation().getZ();
        aruco_tf_stamped.transform.rotation.w = transform.getRotation().getW();
        tf_msg_list.transforms.push_back(aruco_tf_stamped);
        aruco_tf_broadcaster.sendTransform(aruco_tf_stamped);
    }
    m_tf_list_pub.publish(tf_msg_list);
    
    // imshow(OPENCV_WINDOW, copy_image);
    output_image = copy_image;
}


tf2::Vector3 ArucoTracking::CvVector3d2TfVector3(const Vec3d &vec) 
{
    return {vec[0], vec[1], vec[2]};
}


tf2::Quaternion ArucoTracking::CvVector3d2TfQuarternion(const Vec3d &rotation_vector) 
{
    Mat rotation_matrix;
    auto ax = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa = cos(angle * 0.5);
    auto sina = sin(angle * 0.5);
    auto qx = ax * sina / angle;
    auto qy = ay * sina / angle;
    auto qz = az * sina / angle;
    auto qw = cosa;

    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);
    return q;
}

tf2::Transform ArucoTracking::CreateTransform(const Vec3d &translation_vector, const Vec3d &rotation_vector) 
{
    tf2::Transform transform;
    transform.setOrigin(CvVector3d2TfVector3(translation_vector));
    transform.setRotation(CvVector3d2TfQuarternion(rotation_vector));
    return transform;
}


void ArucoTracking::Publish()
{
    // m_aruco_info_pub.publish(ar_msg);
}

}