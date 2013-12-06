#ifndef STEREOCAMERACALIBRATOR_H
#define  STEREOCAMERACALIBRATOR_H

#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"

#include "monocularCameraCalibrator.h"

class stereoCameraCalibrator
{
    monocularcameraCalibrator *leftCamera;
    monocularcameraCalibrator *rightCamera;
    bool stereocalibrationflag;
    cv::Mat R, T, E, F;
	cv::Mat rmap[2][2];
	std::vector<std::vector<cv::Point2f> > stereoimagePoints[2];
	std::vector<std::vector<cv::Point3f> > stereoobjectPoints;
    public:
    stereoCameraCalibrator();
    void stereodetectChessboard(int patternchoise,cv::Size patternsize,cv::Size circlepatternsize);
    void stereosetPoints(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize);
    void stereoCalibratefunction();
    void createSnapshot();
    void savedata();
    void refreshWindow();
    void saveSnapshot();
    void undistort();
};

#endif
