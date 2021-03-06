#include "ros/ros.h"
#include <ros/spinner.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <iostream>
#include <string.h>

#include "uav_msgs/Chat.h"

bool IsNum(const std::string& str) {
    size_t size = str.size();
    
    if (size == 0) return 0;

    for (char const &c : str) {
        if (c == '.' || c == '-' || c == '+') continue;
        if (c < '0' || c > '9') return false;
    }

    return true;
}

bool IsTool(const std::string& str){
    if((str.compare("custom") == 0) || 
        (str.compare("lidar") == 0) ||
        (str.compare("gps") == 0))
    {
        return true;
    }
    return false;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "chat_node");
    ros::NodeHandle nh;

    // package, node, topic name
    std::string nd_name = ros::this_node::getName();
    ros::Publisher char_pub = nh.advertise<uav_msgs::Chat>(nd_name +"/command", 1000);

    ros::Rate loop_rate(10);

    bool is_first = true;
    while (ros::ok())
    {
        if (is_first){
            is_first = false;
            ROS_INFO_STREAM("/****************************************/");
            ROS_INFO_STREAM("/** command: debugging|rc|offboard|    **/");
            ROS_INFO_STREAM("/**          manual|emergy|local|      **/");
            ROS_INFO_STREAM("/**          altctl|land|global_z|     **/");
            ROS_INFO_STREAM("/**          local_z|tool|global|      **/");
            ROS_INFO_STREAM("/**          pause|play|start|         **/");
            ROS_INFO_STREAM("/**                                    **/");
            ROS_INFO_STREAM("/**          wait|undocking|arm|       **/");
            ROS_INFO_STREAM("/**          hovering|flight|          **/");
            ROS_INFO_STREAM("/**          transition|docking|       **/");
            ROS_INFO_STREAM("/**          landing|disarm|takeoff    **/");
            ROS_INFO_STREAM("/****************************************/");
        }

        std::string inputString;
        std::cout << "[command] ";
        std::getline(std::cin, inputString);
        uav_msgs::Chat chat;

        bool is_error = true;
        if((inputString.compare("debugging") == 0) || 
            (inputString.compare("rc") == 0) ||
            (inputString.compare("offboard") == 0) ||
            (inputString.compare("manual") == 0) ||
            (inputString.compare("altctl") == 0) ||
            (inputString.compare("emergy") == 0) ||
            (inputString.compare("land") == 0) ||
            (inputString.compare("global") == 0) ||
            (inputString.compare("wait") == 0) ||
            (inputString.compare("undocking") == 0) ||
            (inputString.compare("arm") == 0) ||
            (inputString.compare("takeoff") == 0) ||
            (inputString.compare("hovering") == 0) ||
            (inputString.compare("flight") == 0) ||
            (inputString.compare("transition") == 0) ||
            (inputString.compare("docking") == 0) ||
            (inputString.compare("disarm") == 0) ||
            (inputString.compare("landing") == 0) ||
            (inputString.compare("pause") == 0) ||
            (inputString.compare("play") == 0) ||
            (inputString.compare("start") == 0) ||
            (inputString.compare("local") == 0))
        {
            std::cout << ("[input] " + inputString) << std::endl;
            chat.msg = inputString;

            char_pub.publish(chat);
        }
        else if(inputString.compare("tool") == 0)
        {
            while (is_error){                    
                std::string inputToolString;
                std::cout << "[tool] ";
                std::getline(std::cin, inputToolString);

                if (IsTool(inputToolString)){
                    std::cout << "[input] tool: " + inputToolString << std::endl;
                    chat.msg = inputString;
                    chat.tool = inputToolString;

                    char_pub.publish(chat);
                    is_error = false;
                }
                else {
                    ROS_ERROR_STREAM("Please input tool (custom, lidar or gps)");
                    is_error = true;
                }
            }
        }
        else if((inputString.compare("global_z") == 0) || 
                (inputString.compare("local_z") == 0))
        {
            while (is_error){
                std::string inputNumString;
                std::cout << "[value] ";
                std::getline(std::cin, inputNumString);

                if (IsNum(inputNumString)){
                    float height = stof(inputNumString);

                    geographic_msgs::GeoPoint geopoint;
                    geometry_msgs::Point point;
                    if ((inputString.compare("global_z") == 0)){
                        std::cout << ("[input] global altitude offset: " + inputNumString) << std::endl;
                        geopoint.altitude = height;
                    }
                    else {
                        std::cout << ("[input] local z offset: " + inputNumString) << std::endl;
                        point.z = height;
                    }
                    chat.msg = inputString;
                    chat.geopoint = geopoint;
                    chat.point = point;

                    char_pub.publish(chat);
                    is_error = false;
                }
                else {
                    ROS_ERROR_STREAM("Please input number");
                    is_error = true;
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
} 