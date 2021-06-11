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
    bool ReadFile(int argc, char *argv[], string& outputFile, int& squaresX, int& squaresY, float& squareLength, 
            float& markerLength, int& dictionaryId, bool& showChessboardCorners, bool& refindStrategy,
            int& camId, Ptr<aruco::DetectorParameters>& detectorParams, int& calibrationFlags, float& aspectRatio);

private:
    bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);
    bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params);
};

}
#endif // __PARSER_H__