#ifndef MONOCULARCAMERACALIBRATOR_H
#define MONOCULARCAMERACALIBRATOR_H

#include "cv.h"
#include "highgui.h"
#include <iostream>
#include <vector>
#include "opencv2/calib3d/calib3d.hpp"


class monocularcameraCalibrator
{
  public:
   int frameHeight;
   int frameWidth;
   
   cv::VideoCapture*  CameraCapture;
   cv::Mat frame;
   cv::Mat greyframe;
   cv::Mat calibratedframe;
  //this vector will be filled by detected corners
  std::vector<cv::Point2f> corners;
  //this will be filled by detected centers
  std::vector<cv::Point2f> centers;
  //indicate whether the complete board was foundor not (=0)
  int FoundFrameIndicator;
  //imagepoints:projection of calibration pattern points
  std::vector<std::vector<cv::Point2f> > imagePoints;
  //objectPoints:real 3d coordinates
  std::vector<std::vector<cv::Point3f> > objectPoints;
  //used to store cameraMatrix and distCoeffs
  cv::Mat cameraMatrix[1], distCoeffs[1];
  //used to indicate if camera is calibrated
  bool calibrationflag;
  public:
  monocularcameraCalibrator(int cameraIndicator);
  ~monocularcameraCalibrator();
  void createSnapshot();
  void refreshWindow();
  void detectChessboard(int patternchoise,cv::Size patternsize,cv::Size circlepatternsize);
  void setPoints(int patternchoise,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize);
  void calibratefunction();
  void undistort();
  void savedata();
  void monocularCalibrate(int patternchoise,cv::Size patternsize,cv::Size circlepatternsize,int squareSize,int circleSize);
};

#endif
