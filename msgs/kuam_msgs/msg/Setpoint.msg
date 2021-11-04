std_msgs/Header header

bool is_global
bool is_setpoint_position

float32 local_height_m
float32 home_altitude_m

# set point
geographic_msgs/GeoPose geopose

geometry_msgs/Twist vel
geometry_msgs/Pose yaw_rate

geometry_msgs/Pose pose

# ego pose
geometry_msgs/Pose ego_pose

LandingState landing_state

geometry_msgs/Pose target_pose
