#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "monocularCameraCalibrator.h"

int main()
{
  int patternchoice;
  //!< Parameters for square patterns
  cv::Size patternsize;
  //!< Number of enclosed corners horizontally on chessboard
  int board_corners_w;
  //!< Number of enclosed corners vertically on chessboard
  int board_corners_h;
  //!< Size of one grid in chessboard
  float squareSize; 
	
  //!< Parameters for circle patterns
  cv::Size circlepatternsize(4,11);
  //!< Number of enclosed centers horizontally
  int board_centers_w;
  //!< Number of enclosed centers vertically
  int board_centers_h;
  //!< Radius of circle in circleboard
  float circleSize;  
	
	std::cout <<"Camera calibration begins!!!!" <<std::endl;
	std::cout<<"*************************************************"<<std::endl;
	std::cout<<"Choose desirable pattern"<<std::endl;
	std::cout<<"Press 1 for square pattern"<<std::endl;
	std::cout<<"Press 2 for circle pattern"<<std::endl;
	std::cin>>patternchoice;
  if(patternchoice==1)
  {
    std::cout<<"Give the number of enclosed corners horizontally"<<std::endl;
    std::cin>>board_corners_w;
    std::cout<<"Give the number of enclosed corners vertically"<<std::endl;
    std::cin>>board_corners_h;
    std::cout<<"Give squaresize"<<std::endl;
    std::cin>>squareSize;
    patternsize=cv::Size(board_corners_w,board_corners_h);
  }
  else
  {
    std::cout<<"Give the number of enclosed centers horizontally"<<std::endl;
    std::cin>>board_centers_w;
    std::cout<<"Give the number of enclosed centers vertically"<<std::endl;
    std::cin>>board_centers_h;
    std::cout<<"Give radius"<<std::endl;
    std::cin>>circleSize;
    circlepatternsize=cv::Size(board_centers_w,board_centers_h);
  }
  monocularcameraCalibrator *leftCamera = new  monocularcameraCalibrator(0);
  monocularcameraCalibrator *rightCamera = new  monocularcameraCalibrator(1);
  leftCamera->monocularCalibrate(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
  rightCamera->monocularCalibrate(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
}
