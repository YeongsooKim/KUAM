#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <iostream>

#include <glob.h> // glob(), globfree()
#include <stdexcept>
#include <sstream>
#include <string.h> // memset()
#include <string>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

vector<string> Glob(const string& pattern);
bool HasEnding (std::string const &fullString, std::string const &ending);

int frame_rate = 5;
string path = "/home/ys/dataset/au_air/frame_20190905111947/*";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_img_stream");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    p_nh.getParam("frame_rate", frame_rate);
    p_nh.getParam("file_path", path);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image", 1);
    
    auto file_list = Glob(path);
    vector<string> file_list_jpg;
    for (auto file : file_list){
        if (HasEnding(file, "png")){
            file_list_jpg.push_back(file);
        }
    }

    ros::Rate loop_rate(frame_rate);

    while(true){
        for (auto file : file_list_jpg){
            cout << file << endl;

            cv::Mat image = cv::imread(file, CV_LOAD_IMAGE_COLOR);
            loop_rate.sleep();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            pub.publish(msg);
        }
    }
    
    return 0;
}


vector<string> Glob(const string& pattern) {
    using namespace std;

    // glob struct resides on the stack
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));

    // do the glob operation
    int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    if(return_value != 0) {
        globfree(&glob_result);
        stringstream ss;
        ss << "glob() failed with return_value " << return_value << endl;
        throw runtime_error(ss.str());
    }

    // collect all the filenames into a list<string>
    vector<string> filenames;
    for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
        filenames.push_back(string(glob_result.gl_pathv[i]));
    }

    // cleanup
    globfree(&glob_result);

    // done
    return filenames;
}

bool HasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}