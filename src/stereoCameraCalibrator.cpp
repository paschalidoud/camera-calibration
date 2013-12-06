#include "stereoCameraCalibrator.h"

stereoCameraCalibrator::stereoCameraCalibrator()
{
	leftCamera = new  monocularcameraCalibrator(0);
	rightCamera = new  monocularcameraCalibrator(0);

    stereocalibrationflag=false;
   
}

void stereoCameraCalibrator::createSnapshot()
{
	    //retrieve data(shnapshot) from camera
        leftCamera->CameraCapture->grab();
        rightCamera->CameraCapture->grab();
        leftCamera->CameraCapture->retrieve(leftCamera->frame);
        rightCamera->CameraCapture->retrieve(rightCamera->frame);
        //turn image into gray
        cvtColor(leftCamera->frame, leftCamera->greyframe, CV_RGB2GRAY);
        cvtColor(rightCamera->frame, rightCamera->greyframe, CV_RGB2GRAY);
}

void stereoCameraCalibrator::stereodetectChessboard(int patternchoice,cv::Size patternsize,cv::Size circlepatternsize)
{
     int patternflagleft=2;
     int patternflagright=2;

  if(patternchoice==1)
  {
    //find and draw corners for the camera's snapshot
      patternflagleft=cv::findChessboardCorners(leftCamera->greyframe,patternsize,leftCamera->corners,
           cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE/*+CALIB_CB_FAST_CHECK*/);
     patternflagleft=findChessboardCorners(rightCamera->greyframe,patternsize,rightCamera->corners,
           cv::CALIB_CB_ADAPTIVE_THRESH+cv::CALIB_CB_NORMALIZE_IMAGE/*+CALIB_CB_FAST_CHECK*/);
  // improve the found corners' coordinate accuracy for chessboard
  if( patternflagleft==0 && patternflagright==0 &&(leftCamera->corners.size()!=0)&&(rightCamera->corners.size()!=0))
  {
      cornerSubPix(leftCamera->greyframe,leftCamera->corners, cv::Size(11, 11), cv::Size(-1, -1),
                   cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      cornerSubPix(rightCamera->greyframe,rightCamera->corners, cv::Size(11, 11), cv::Size(-1, -1),
                   cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  }
    //draw the corners
       cv::drawChessboardCorners(leftCamera->frame,patternsize, cv::Mat(leftCamera->corners), patternflagleft);
       cv::drawChessboardCorners(rightCamera->frame,patternsize, cv::Mat(rightCamera->corners), patternflagright);
  }
  else if(patternchoice==2)
  {
      //find and draw corners for the left camera's snapshot
      patternflagleft= cv::findCirclesGrid(leftCamera->greyframe,circlepatternsize,leftCamera->centers,
                cv::CALIB_CB_ASYMMETRIC_GRID/*+CALIB_CB_CLUSTERING*/);
    patternflagright= cv::findCirclesGrid(rightCamera->greyframe,circlepatternsize,rightCamera->centers,
                cv::CALIB_CB_ASYMMETRIC_GRID/*+CALIB_CB_CLUSTERING*/);
    //draw the corners
       cv::drawChessboardCorners(leftCamera->frame,circlepatternsize,cv::Mat(leftCamera->centers),patternflagleft);
       cv::drawChessboardCorners(rightCamera->frame,circlepatternsize,cv::Mat(rightCamera->centers),patternflagright);
  }
  else
  {
      std::cout<<"There is not such a pattern choise"<<std::endl;
  }
}

void stereoCameraCalibrator::stereoCalibratefunction()
{
    std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	cv::Size imageSize=leftCamera->greyframe.size();
    double cameraReprojectionError;
    std::cout<<"Calibration for stereo begins!"<<std::endl;
    cameraReprojectionError = cv::stereoCalibrate(leftCamera->objectPoints, stereoimagePoints[0], stereoimagePoints[1],
					leftCamera->cameraMatrix[0],leftCamera->distCoeffs[0],
					rightCamera->cameraMatrix[0],rightCamera->distCoeffs[0],
					imageSize, R, T, E, F,
					cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),CV_CALIB_FIX_INTRINSIC);
    cv::FileStorage filestream("fundamental_essential.yml", CV_STORAGE_WRITE);
	if(filestream.isOpened() )
	{
		filestream << "R" << R << "T" << T << "E"<< E <<"F" << F;
		filestream.release();
	}
	else
	{
		std::cout << "Error: can not save the fundamental and essential matrix"<<std::endl;
    }
	stereocalibrationflag=true;
    std::cout<<"Calibration for stereo ended! RMS error:" <<cameraReprojectionError<<std::endl;
    this->savedata();
    this->undistort();
}
void stereoCameraCalibrator::savedata()
{
   	cv::Size imageSize=leftCamera->greyframe.size();
	cv::FileStorage fs("extrinsics_eye.yml", CV_STORAGE_WRITE);
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];
	cv::stereoRectify(leftCamera->cameraMatrix[0],leftCamera->distCoeffs[0],
					rightCamera->cameraMatrix[0],rightCamera->distCoeffs[0],
			imageSize, R, T, R1, R2, P1, P2, Q,
			cv::CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	if( fs.isOpened() )
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
	{
		std::cout << "Error: can not save the extrinsic parameters"<<std::endl;
    }
	std::cout << "Saved parameters!"<<std::endl;
}
void stereoCameraCalibrator::refreshWindow()
{
    cv::imshow( "left snapshot", leftCamera->frame );
	cv::imshow( "right snapshot", rightCamera->frame);
}

void stereoCameraCalibrator::undistort()
{
    cv::Mat R1, R2, P1, P2, Q;
    cv::Size imageSize=leftCamera->greyframe.size();
	cv::Rect roi1, roi2;

	cv::stereoRectify( leftCamera->cameraMatrix[0],leftCamera-> distCoeffs[0],
               rightCamera-> cameraMatrix[0],rightCamera-> distCoeffs[0], imageSize, R, T, R1, R2, P1, P2, Q,
	cv::CALIB_ZERO_DISPARITY, -1, imageSize, &roi1, &roi2 );

	/*
	 *The initUndistortRectifyMap computes the undistortion and rectification transformation map
	 *and returns rmap to be used in remap function
     */
	cv::initUndistortRectifyMap(leftCamera->cameraMatrix[0],leftCamera-> distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(rightCamera-> cameraMatrix[0],rightCamera-> distCoeffs[0], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
    cv::remap(leftCamera->frame, leftCamera->calibratedframe, rmap[0][0], rmap[0][1], cv::INTER_LINEAR);
    cv::remap(rightCamera->frame,rightCamera->calibratedframe, rmap[1][0], rmap[1][1], cv::INTER_LINEAR);
}

int main()
{

	int patternchoice;
	
	//Parameters for square patterns
	cv::Size patternsize;
	//enclosed corners horizontally on chessboard
	int board_corners_w;
	//enclosed corners vertically on chessboard
	int board_corners_h;
	//size of one grid
	float squareSize; 
	
	//Parameters for circle patterns
	 cv::Size circlepatternsize(4,11);
	 //enclosed centers horizontally
     int board_centers_w;
     //enclosed centers vertically
     int board_centers_h;
     //radius
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
