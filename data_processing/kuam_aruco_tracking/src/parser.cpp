#include <kuam_aruco_tracking/parser.h>
#include <fstream>
#include <iostream>
#include <vector>

#include "opencv2/aruco/dictionary.hpp"
#include <ros/ros.h>

using namespace cv;
using namespace std;

namespace kuam
{

namespace {
const char* about = "Basic marker detection";
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }";
}


Parser::Parser()
{}

Parser::~Parser()
{}

bool Parser::ReadFile(string file_path, bool& do_estimate_pose, 
                bool& show_rejected, float& marker_length, 
                cv::Ptr<cv::aruco::DetectorParameters>& detector_params, 
                cv::Ptr<cv::aruco::Dictionary>& dictionary, 
                cv::Mat& cam_matrix, cv::Mat& dist_coeffs)
{
	string parser = file_path + "aruco_parser.txt";

	// read File
	int argc = 1;
	vector<string> lines;
	lines.push_back("aruco_tracking");
	ifstream openFile(parser.data());
	if( openFile.is_open() ){
		string line;
		while(getline(openFile, line)){
			lines.push_back(line);
			argc++;
		}
		openFile.close();
	}
    else{
		ROS_ERROR_STREAM("Fail to open file in reading");
        return false;
    }

	const char* argv[argc];
	for (int i = 0; i < argc; i++){
		argv[i] = lines[i].c_str();
	}

	CommandLineParser cv_parser(argc, argv, keys);
    cv_parser.about(about);

    if(argc < 2) {
        cv_parser.printMessage();
        return 0;
    }

    int dictionaryId = cv_parser.get<int>("d");
    show_rejected = cv_parser.has("r");
    do_estimate_pose = cv_parser.has("c");
    marker_length = cv_parser.get<float>("l");

    detector_params = aruco::DetectorParameters::create();
    if(cv_parser.has("dp")) {
        bool readOk = readDetectorParameters(cv_parser.get<string>("dp"), detector_params);
        if(!readOk) {
            ROS_ERROR_STREAM("Invalid detector parameters file");
            return false;
        }
    }
    detector_params->doCornerRefinement = true; // do corner refinement in markers

    int camId = cv_parser.get<int>("ci");

    String video;
    if(cv_parser.has("v")) {
        video = cv_parser.get<String>("v");
    }

    if(!cv_parser.check()) {
        cv_parser.printErrors();
        return 0;
    }

    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    cam_matrix, dist_coeffs;
    if(do_estimate_pose) {
        bool readOk = readCameraParameters(cv_parser.get<string>("c"), cam_matrix, dist_coeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return false;
        }
    }

    return true;    
}

bool Parser::readCameraParameters(string filename, Mat &cam_matrix, Mat &dist_coeffs)
{
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> cam_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    return true;
}

bool Parser::readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) 
{
    FileStorage fs(filename, FileStorage::READ);

    if(!fs.isOpened())
        return false;

    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    
    return true;
}
}