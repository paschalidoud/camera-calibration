#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "monocularCameraCalibrator.h"

int main()
{
  //!< Initialize camera id device
  monocularcameraCalibrator *leftCamera = new  monocularcameraCalibrator(0);
  monocularcameraCalibrator *rightCamera = new  monocularcameraCalibrator(1);
  stereoCameraCalibrator *stereoCamera = new  stereoCameraCalibrate(0,1);
    
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
  
  std::cout<<"Choose camera for calibration"<<std::endl;
  std::cout<<"If you want to calibrate left camera press 1"<<std::endl;
  std::cout<<"If you want to calibrate right camera press 2"<<std::endl;
  std::cout<<"If you want to calibrate stereo camera press 3"<<std::endl;
  std::cin>>choice;
  while(1)
  {
    while(choice==1)
    {
      std::cout<<"Left camera's calibration begins"<<std::endl;
      while(1)
      {
        leftCamera->monocularCalibrate(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
      }
      std::cout<<"If you want to change camera: press 2 for right and 3 for stereo"<<std::endl;
      std::cin>>choice;
    } 
    while(choice==2)
    {
      std::cout<<"Right camera's calibration begins"<<std::endl;
      while(1)
      {   
        rightCamera->monocularCalibrate(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
      }
      std::cout<<"If you want to change camera: press 1 for left and 3 for stereo"<<std::endl;
      std::cin>>choice;
    }
    while(choice==3)
    {
      std::cout<<"Stereo camera's calibration begins"<<std::endl;
      while(1)
      {
        stereoCamera->stereoCalibrate(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
      }
      std::cout<<"If you want to change camera: press 1 for left and 2 for right"<<std::endl;
      std::cin>>choice;
    }      
  }  
}
