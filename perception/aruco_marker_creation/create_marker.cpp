#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <iostream>

int main()
{
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    std::string marker_name = "marker";
    std::string extension = ".png";

    int id = 0;
    for (; id < 50; id++){
        cv::aruco::drawMarker(dictionary, id, 200, markerImage, 1);
        
        std::string id_str = std::to_string(id);
        std::string marker = marker_name + id_str + extension;

        cv::imwrite(marker, markerImage);
    }
}