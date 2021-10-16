#ifndef __ARUCO_TRACKING_UTIL_MARKER_H__
#define __ARUCO_TRACKING_UTIL_MARKER_H__

#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <kuam_msgs/ArucoState.h>
#include <kuam_msgs/ArucoStates.h>

#include <kuam_aruco_tracking/parser.h>
#include <kuam_aruco_tracking/target.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>

#include <kuam_aruco_tracking/target.h>

namespace kuam{
class UtilMarker
{
public:
    UtilMarker();
    ~UtilMarker();

private:
    tf2::Vector3 CvVector3d2TfVector3(const Vec3d &vec) ;
    tf2::Quaternion CvVector3d2TfQuarternion(const Vec3d &rotation_vector) ;

public:
    bool Convert2CVImg(const sensor_msgs::Image::ConstPtr &img_ptr, cv_bridge::CvImagePtr& cv_ptr);
    tf2::Transform CreateTransform(const Vec3d &translation_vector, const Vec3d &rotation_vector) ;
    void GetDetectedId(vector<int>& detected_ids, vector<int>& undetected_ids, const vector<int> src_ids, const vector<int> trg_ids);
    void GetIdsnCorners(const vector<int> ref_ids, const vector<int> input_ids, const vector<vector<Point2f>> input_corners,
                            vector<int>& output_ids, vector<vector<Point2f>>& output_corners);
    void GetNoiseIndexes(vector<int>& noises, const vector<int> ids, vector<Target>& targets);
    vector<kuam_msgs::ArucoState> GetMarkerState(const vector<int>& target_ids, const kuam_msgs::ArucoStates& ac_states_msg);
    void EraseIdnCorner(const vector<int> erase_ids, vector<int>& ids, vector<vector<Point2f>>& corners);
    geometry_msgs::Quaternion ZProjection(const double x, const double y, const double z, const double w);
};
}

#endif // __ARUCO_TRACKING_UTIL_MARKER_H__