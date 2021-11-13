#include <kuam_aruco_tracking/utils/util_marker.h>
#include <sstream>
#include <math.h>

namespace kuam
{
UtilMarker::UtilMarker()
{}

UtilMarker::~UtilMarker() {}

bool UtilMarker::Convert2CVImg(const sensor_msgs::Image::ConstPtr &img_ptr, cv_bridge::CvImagePtr& cv_ptr)
{
    try {
        cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);
        return true;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("[aruco_tracking] cv_bridge exception: %s", e.what());
        return false;
    }
}

tf2::Vector3 UtilMarker::CvVector3d2TfVector3(const Vec3d &vec) 
{
    return {vec[0], vec[1], vec[2]};
}

tf2::Quaternion UtilMarker::CvVector3d2TfQuarternion(const Vec3d &rotation_vector) 
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

tf2::Transform UtilMarker::CreateTransform(const Vec3d &translation_vector, const Vec3d &rotation_vector) 
{
    tf2::Transform transform;
    transform.setOrigin(CvVector3d2TfVector3(translation_vector));
    transform.setRotation(CvVector3d2TfQuarternion(rotation_vector));
    return transform;
}

void UtilMarker::GetDetectedId(vector<int>& detected_ids, vector<int>& undetected_ids,
                    const vector<int> src_ids, const vector<int> trg_ids)
{
    for (auto trg_id : trg_ids){
        auto it = find(src_ids.begin(), src_ids.end(), trg_id);
        if (it == src_ids.end()){
            undetected_ids.push_back(trg_id);
        }
        else{
            detected_ids.push_back(trg_id);
        }
    }    
}

void UtilMarker::GetIdsnCorners(const vector<int> ref_ids, const vector<int> input_ids, const vector<vector<Point2f>> input_corners,
                            vector<int>& output_ids, vector<vector<Point2f>>& output_corners)
{
    for (int index = 0; index < input_ids.size(); index++){
        auto it = find(ref_ids.begin(), ref_ids.end(), input_ids[index]);
        if (it == ref_ids.end()){
            continue;
        }

        output_corners.push_back(input_corners[index]);
        output_ids.push_back(input_ids[index]);
    }
}

void UtilMarker::GetNoiseIndexes(vector<int>& noises, const vector<int> ids, vector<Target>& targets)
{
    for (int index = 0; index < ids.size(); index++){
        for (auto& target : targets){
            if (target.GetId() != ids[index]){
                continue;
            }

            bool is_noise = target.CheckNoise();
            
            if (is_noise){
                noises.push_back(index);
            }
            break;
        }
    }
}

vector<kuam_msgs::ArucoState> UtilMarker::GetMarkerStates(const vector<int>& target_ids, const kuam_msgs::ArucoStates& ac_states_msg)
{
    vector<kuam_msgs::ArucoState> markers;
    for (auto id : target_ids){
        for (auto ac_state : ac_states_msg.aruco_states){
            if (ac_state.id == id && ac_state.is_detected){
                markers.push_back(ac_state);
                break;
            }
        }
    }

    return markers;
}

vector<kuam_msgs::ArucoState> UtilMarker::GetMarkerStates(const vector<int>& target_ids, const vector<Target>& targets,
                                map<int, FrequencyDegree>& id2frequencyDegrees, string camera_frame_id, int estimating_method)
{
    vector <kuam_msgs::ArucoState> aruco_states;
    for (auto id : target_ids){
        for (auto target : targets){
            if (id != target.GetId()){
                continue;
            }

            kuam_msgs::ArucoState ac_state_msg;
            ac_state_msg.header.frame_id = camera_frame_id;
            ac_state_msg.header.stamp = ros::Time::now();
            ac_state_msg.id = target.GetId();

            if (target.GetIsDetected()){
                ac_state_msg.is_detected = true;
                
                ac_state_msg.pose.position.x = target.GetX(estimating_method);
                ac_state_msg.pose.position.y = target.GetY(estimating_method);
                ac_state_msg.pose.position.z = target.GetZ(estimating_method);
                
                ac_state_msg.pose.orientation.x = target.GetQX();
                ac_state_msg.pose.orientation.y = target.GetQY();
                ac_state_msg.pose.orientation.z = target.GetQZ();
                ac_state_msg.pose.orientation.w = target.GetQW();

                ac_state_msg.frequency_degree = id2frequencyDegrees.find(ac_state_msg.id)->second.GetValue();
            }
            else{
                ac_state_msg.is_detected = false;
            }
            aruco_states.push_back(ac_state_msg);

            break;
        }
    }

    return aruco_states;
}

void UtilMarker::EraseIdnCorner(const vector<int> erase_ids, vector<int>& ids, vector<vector<Point2f>>& corners)
{
    vector<int> indexes;
    for (auto erase_id : erase_ids){
        for (int index = 0; index < ids.size(); index++){
            if (ids[index] == erase_id){
                indexes.push_back(index);
                break;
            }
        }
    }

    if (!indexes.empty()){
        sort(indexes.begin(), indexes.end(), greater<int>());

        for (auto index : indexes){
            corners.erase(corners.begin() + index);
            ids.erase(ids.begin() + index);
        }
    }
}

geometry_msgs::Quaternion UtilMarker::ZProjection(const double x, const double y, const double z, const double w)
{
    // Convert quaternion to euler
	tf2::Quaternion from_quat(x, y, z, w);
	double roll, pitch, yaw;
	tf2::Matrix3x3(from_quat).getRPY(roll, pitch, yaw);

    // Remove roll and pitch element
    // Convert euler to quaternion and return it
    tf2::Quaternion to_quat;
    to_quat.setRPY(M_PI, 0.0, yaw);

    geometry_msgs::Quaternion q;
    q.x = to_quat.x();
    q.y = to_quat.y();
    q.z = to_quat.z();
    q.w = to_quat.w();

    return q;
}
}