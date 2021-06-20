#ifndef __PARSER_H__
#define __PARSER_H__

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <string>

using namespace std;
using namespace cv;

namespace kuam
{

class Parser
{
public:
    Parser();
    virtual ~Parser();
    bool ReadFile(int dictionaryId, string calib_path, string detector_params_path, 
                cv::Ptr<cv::aruco::DetectorParameters>& detector_params, 
                cv::Ptr<cv::aruco::Dictionary>& dictionary, cv::Mat& cam_matrix, cv::Mat& dist_coeffs);

private:
    string m_aruco_parser_param; // Dictionary

private:
    bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);
    bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params);
};

}
#endif // __PARSER_H__