#include "cv.h"
#include "highgui.h"
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"

#define BLUR_SHIFT 24

using namespace std;
using namespace cv;

class cameraCalibrate
{
  public:
   VideoCapture*  CameraCapture;
   Mat* frame;
   Mat* greyframe;
   Mat* calibratedframe;
  //this vector will be filled by detected corners
  vector<Point2f> corners;
  //this will be filled by detected centers
  vector<Point2f> centers;
  //indicate whether the complete board was foundor not (=0)
  int FoundFrameIndicator;
  //imagepoints:projection of calibration pattern points
  vector<vector<Point2f> > imagePoints[1];
  //objectPoints:real 3d coordinates
  vector<vector<Point3f> > objectPoints;
  //used to store cameraMatrix and distCoeffs
  Mat cameraMatrix[1], distCoeffs[1];
  //used to indicate if camera is calibrated
  bool calibrationflag;
  public:
  cameraCalibrate(int cameraIndicator);
  ~cameraCalibrate();
  void createSnapshot();
  void refreshWindow();
  void detectChessboard(int patternchoise,Size patternsize,Size circlepatternsize);
  void setPoints(int patternchoise,Size patternsize,Size circlepatternsize,int squareSize,int circleSize);
  void saveSnapshot();
  void calibratefunction();
  void undistort();
  void savedata();
};

//***********camera class functions**************************
//constructor,initialize neccesary parameters
cameraCalibrate::cameraCalibrate(int cameraIndicator)
{
    cout<<"Camera constructor"<<endl;
    int height=480;
    int width=640;
    //indicate whether the complete board was foundor not (=0)
    FoundFrameIndicator=0;
    CameraCapture=new VideoCapture(cameraIndicator);
    if(!CameraCapture->isOpened())
    {
       cout<<"Selected camera is not initialized"<<endl;
    }
    frame =new  Mat(height,width,CV_8UC1);
    calibratedframe=new Mat(height,width,CV_8UC1);
    greyframe=new Mat(height,width,CV_8UC1);
    CameraCapture->set(CV_CAP_PROP_FRAME_WIDTH, 640);
    CameraCapture->set(CV_CAP_PROP_FRAME_HEIGHT,480);
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    distCoeffs[0] = Mat::zeros(8, 1, CV_64F);
    calibrationflag=false;
    namedWindow("snapshot",CV_WINDOW_AUTOSIZE);
	namedWindow("calibrated snapshot",CV_WINDOW_AUTOSIZE);
}

//with this function we take snapshot
void cameraCalibrate::createSnapshot()
{
   // cout<<"createSnapshot function"<<endl;
   if(CameraCapture->isOpened()==false)
	{
        cout<<"camera is not initialized"<<endl;
	}
	else
	{
	    //retrieve data(shnapshot) from camera
        CameraCapture->grab();
        CameraCapture->retrieve(*frame);
        //turn image into gray
        cvtColor(*frame, *greyframe, CV_RGB2GRAY);
	}
}

 void cameraCalibrate::refreshWindow()
 {
            imshow("snapshot",*frame);
            if(!calibratedframe->data)
            {
                imshow("calibrated snapshot",*greyframe);
            }
            else
            {
                imshow("calibrated snapshot",*calibratedframe);
            }
 }

 void cameraCalibrate::detectChessboard(int patternchoice,Size patternsize,Size circlepatternsize)
{
  int patternflag=2;
//  cout<<"detectChessboard function"<<endl;
  if(patternchoice==1)
  {

    //find and draw corners for the camera's snapshot
      patternflag=findChessboardCorners(*greyframe,patternsize,corners,
           CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE/*+CALIB_CB_FAST_CHECK*/);
  // improve the found corners' coordinate accuracy for chessboard
  if(patternflag==0 && (corners.size()!=0))
  {
      cornerSubPix(*greyframe,corners, Size(11, 11), Size(-1, -1),
                   TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  }
    //draw the corners
       drawChessboardCorners(*frame,patternsize, Mat(corners),patternflag);
  }
  else if(patternchoice==2)
  {
      //find and draw corners for the left camera's snapshot
      patternflag= findCirclesGrid(*greyframe,circlepatternsize,centers,
                CALIB_CB_ASYMMETRIC_GRID/*+CALIB_CB_CLUSTERING*/);
    //draw the corners
       drawChessboardCorners(*frame,circlepatternsize, Mat(centers),patternflag);
  }
  else
  {
      cout<<"There is not such a pattern choise"<<endl;
  }

}
/*
*in this function we set object kai image points!object points referr to the real 3d coordinates of pattern points
*in the calibration pattern,whereas image points are 2d cordinates and refer to the projections of calibration pattern
*points
*/
void cameraCalibrate::setPoints(int patternchoise,Size patternsize,Size circlepatternsize,int squareSize,int circleSize)
{
    //set object points
    vector<Point3f> obj;
    if(patternchoise==1)
    {
        for( int j = 0; j < patternsize.height; j++ )
        {
          for( int k = 0; k < patternsize.width; k++ )
          {
           obj.push_back(Point3f(float(j*squareSize), float(k*squareSize), 0));
          }
        }
    }
    else
    {
        for( int i = 0; i < circlepatternsize.height; i++ )
        {
            for( int j = 0; j < circlepatternsize.width; j++ )
            {
            obj.push_back(Point3f(float((2*j + i % 2)*circleSize), float(i*circleSize), 0));
            }
        }
    }
    objectPoints.push_back(obj);
   //set imagePoints
   if(patternchoise==1)
    {
        imagePoints[0].push_back(corners);
    }
    else
    {
       imagePoints[0].push_back(centers);
    }
    cout<<"Object and Image Points are stored"<<endl;
}
void cameraCalibrate::calibratefunction()
{
    Size imageSize=greyframe->size();
	vector<Mat> rvecs;
	vector<Mat> tvecs;
    double cameraReprojectionError;
    cout<<"Camera Calibration started"<<endl;

    cameraReprojectionError = calibrateCamera(objectPoints, imagePoints[0], imageSize, cameraMatrix[0],
            distCoeffs[0], rvecs, tvecs,CV_CALIB_FIX_K4| CV_CALIB_FIX_K5);
    cout<<"Calibration for left camera ended! RMS error:" <<cameraReprojectionError<<"\n";
    calibrationflag=true;
    cout<<"Camera is calibrated"<<endl;
    this->savedata();
    this->undistort();
}
void cameraCalibrate::undistort()
{
    Size imageSize=greyframe->size();
    Mat map1, map2;
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], Mat(),
        getOptimalNewCameraMatrix(cameraMatrix[0], distCoeffs[0], imageSize, 1, imageSize, 0),imageSize, CV_16SC2, map1, map2);
    remap(*greyframe, *calibratedframe, map1, map2, INTER_LINEAR);
}
void cameraCalibrate::savedata()
{
	FileStorage fs("intrinsics_eye.yml", CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "cameraMatrix" << cameraMatrix[0] << "distortionCoefficientMatrix" << distCoeffs[0];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";
}
void cameraCalibrate::saveSnapshot()
{
    imwrite( "/home/despoina/Gray_Image1.png", *greyframe);
}
cameraCalibrate::~cameraCalibrate()
{
    destroyWindow("snapshot");
	destroyWindow("calibrated snapshot");
}
class stereoCameraCalibrate
{
    cameraCalibrate *leftCamera;
    cameraCalibrate *rightCamera;
    bool stereocalibrationflag;
    Mat R, T, E, F;
	Mat rmap[2][2];
	vector<vector<Point2f> > stereoimagePoints[2];
	vector<vector<Point3f> > stereoobjectPoints;
    public:
    stereoCameraCalibrate(cameraCalibrate *obj1,cameraCalibrate *obj2);
    void stereodetectChessboard(int patternchoise,Size patternsize,Size circlepatternsize);
    void stereosetPoints(int patternchoice,Size patternsize,Size circlepatternsize,int squareSize,int circleSize);
    void stereoCalibratefunction();
    void stereoCreateSnapshot();
    void savedata();
    void refreshWindow();
    void saveSnapshot();
    void undistort();
};

//***********camera class functions**************************
//constructor,initialize neccesary parameters
stereoCameraCalibrate::stereoCameraCalibrate(cameraCalibrate *obj1,cameraCalibrate *obj2)
{
    leftCamera=obj1;
    rightCamera=obj2;
    if(leftCamera->calibrationflag==false)
    {
        cout<<"Left camera has not been calibrated yet"<<endl;
    }
    if(rightCamera->calibrationflag==false)
    {
        cout<<"Right camera has not been calibrated yet"<<endl;
    }
    stereocalibrationflag=false;
    namedWindow("left snapshot",CV_WINDOW_AUTOSIZE);
    namedWindow("right snapshot",CV_WINDOW_AUTOSIZE);
}
void stereoCameraCalibrate::stereoCreateSnapshot()
{
    // cout<<"createSnapshot function"<<endl;
   if(leftCamera->CameraCapture->isOpened()==false)
	{
        cout<<"left camera is not initialized"<<endl;
	}
	 if(rightCamera->CameraCapture->isOpened()==false)
	{
        cout<<"right camera is not initialized"<<endl;
	}
	else
	{
	    //retrieve data(shnapshot) from camera
        leftCamera->CameraCapture->grab();
        rightCamera->CameraCapture->grab();
        leftCamera->CameraCapture->retrieve(*(leftCamera->frame));
        rightCamera->CameraCapture->retrieve(*(rightCamera->frame));
        //turn image into gray
        cvtColor(*(leftCamera->frame), *(leftCamera->greyframe), CV_RGB2GRAY);
        cvtColor(*(rightCamera->frame), *(rightCamera->greyframe), CV_RGB2GRAY);
	}
}
void stereoCameraCalibrate::stereodetectChessboard(int patternchoice,Size patternsize,Size circlepatternsize)
{
     int patternflagleft=2;
     int patternflagright=2;
//  cout<<"detectChessboard function"<<endl;
  if(patternchoice==1)
  {
    //find and draw corners for the camera's snapshot
      patternflagleft=findChessboardCorners(*(leftCamera->greyframe),patternsize,leftCamera->corners,
           CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE/*+CALIB_CB_FAST_CHECK*/);
     patternflagleft=findChessboardCorners(*(rightCamera->greyframe),patternsize,rightCamera->corners,
           CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE/*+CALIB_CB_FAST_CHECK*/);
  // improve the found corners' coordinate accuracy for chessboard
  if( patternflagleft==0 && patternflagright==0 &&(leftCamera->corners.size()!=0)&&(rightCamera->corners.size()!=0))
  {
      cornerSubPix(*(leftCamera->greyframe),leftCamera->corners, Size(11, 11), Size(-1, -1),
                   TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      cornerSubPix(*(rightCamera->greyframe),rightCamera->corners, Size(11, 11), Size(-1, -1),
                   TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  }
    //draw the corners
       drawChessboardCorners(*(leftCamera->frame),patternsize, Mat(leftCamera->corners), patternflagleft);
       drawChessboardCorners(*(rightCamera->frame),patternsize, Mat(rightCamera->corners), patternflagright);
  }
  else if(patternchoice==2)
  {
      //find and draw corners for the left camera's snapshot
      patternflagleft= findCirclesGrid(*(leftCamera->greyframe),circlepatternsize,leftCamera->centers,
                CALIB_CB_ASYMMETRIC_GRID/*+CALIB_CB_CLUSTERING*/);
    patternflagright= findCirclesGrid(*(rightCamera->greyframe),circlepatternsize,rightCamera->centers,
                CALIB_CB_ASYMMETRIC_GRID/*+CALIB_CB_CLUSTERING*/);
    //draw the corners
       drawChessboardCorners(*(leftCamera->frame),circlepatternsize, Mat(leftCamera->centers),patternflagleft);
       drawChessboardCorners(*(rightCamera->frame),circlepatternsize, Mat(rightCamera->centers),patternflagright);
  }
  else
  {
      cout<<"There is not such a pattern choise"<<endl;
  }
}
void stereoCameraCalibrate::stereosetPoints(int patternchoice,Size patternsize,Size circlepatternsize,int squareSize,int circleSize)
{
    //set object points
    vector<Point3f> obj;
    if(patternchoice==1)
    {
        for( int j = 0; j < patternsize.height; j++ )
        {
          for( int k = 0; k < patternsize.width; k++ )
          {
           obj.push_back(Point3f(float(j*squareSize), float(k*squareSize), 0));
          }
        }
    }
    else
    {
        for( int i = 0; i < circlepatternsize.height; i++ )
        {
            for( int j = 0; j < circlepatternsize.width; j++ )
            {
            obj.push_back(Point3f(float((2*j + i % 2)*circleSize), float(i*circleSize), 0));
            }
        }
    }
    //objectPoints is the same for both cameras
    stereoobjectPoints.push_back(obj);
   //set imagePoints
   if(patternchoice==1)
    {
        stereoimagePoints[0].push_back(leftCamera->corners);
         stereoimagePoints[1].push_back(rightCamera->corners);
    }
    else
    {
       stereoimagePoints[0].push_back(leftCamera->centers);
       stereoimagePoints[1].push_back(rightCamera->centers);
    }
    cout<<"Object and Image Points are stored"<<endl;
}
void stereoCameraCalibrate::stereoCalibratefunction()
{
    vector<Mat> rvecs;
	vector<Mat> tvecs;
	Size imageSize=leftCamera->greyframe->size();
    double cameraReprojectionError;
        cout<<"Left and Right camera are already calibrated"<<endl;
        cout<<"Calibration for stereo started!\n";
        cameraReprojectionError = stereoCalibrate(leftCamera->objectPoints, stereoimagePoints[0], stereoimagePoints[1],
					leftCamera->cameraMatrix[0],leftCamera->distCoeffs[0],
					rightCamera->cameraMatrix[0],rightCamera->distCoeffs[0],
					imageSize, R, T, E, F,
					TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
					CV_CALIB_FIX_INTRINSIC);
    FileStorage filestream("fundamental_essential.yml", CV_STORAGE_WRITE);
	if(filestream.isOpened() )
	{
		filestream << "R" << R << "T" << T << "E"<< E <<"F" << F;
		filestream.release();
	}
	else
	{
		cout << "Error: can not save the fundamental and essential matrix\n";
    }
		stereocalibrationflag=true;
        cout<<"Calibration for stereo ended! RMS error:" <<cameraReprojectionError<<"\n";
    this->savedata();
    this->undistort();
}
void stereoCameraCalibrate::savedata()
{
   	Size imageSize=leftCamera->greyframe->size();
	FileStorage fs("extrinsics_eye.yml", CV_STORAGE_WRITE);
	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];
	stereoRectify(leftCamera->cameraMatrix[0],leftCamera->distCoeffs[0],
					rightCamera->cameraMatrix[0],rightCamera->distCoeffs[0],
			imageSize, R, T, R1, R2, P1, P2, Q,
			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
	if( fs.isOpened() )
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
	{
		cout << "Error: can not save the extrinsic parameters\n";
    }
	cout << "Saved parameters!\n";
}
void stereoCameraCalibrate::refreshWindow()
{
    imshow( "left snapshot", *(leftCamera->frame) );
	imshow( "right snapshot", *(rightCamera->frame) );
}
void stereoCameraCalibrate::saveSnapshot()
{
    leftCamera->saveSnapshot();
    rightCamera->saveSnapshot();
}

void stereoCameraCalibrate::undistort()
{
    Mat R1, R2, P1, P2, Q;
    Size imageSize=leftCamera->greyframe->size();
	Rect roi1, roi2;

	stereoRectify( leftCamera->cameraMatrix[0],leftCamera-> distCoeffs[0],
               rightCamera-> cameraMatrix[0],rightCamera-> distCoeffs[0], imageSize, R, T, R1, R2, P1, P2, Q,
	CALIB_ZERO_DISPARITY, -1, imageSize, &roi1, &roi2 );

 //initUndistortRectifyMap->computes the undistortion and rectification transformation map
 //epistrefei ro rmap gia na xrhsimopoihthei stin remap
	initUndistortRectifyMap(leftCamera->cameraMatrix[0],leftCamera-> distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(rightCamera-> cameraMatrix[0],rightCamera-> distCoeffs[0], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
//	remap(leftCamera->frame, leftCamera->calibratedframe, rmap[0][0], rmap[0][1], INTER_LINEAR);
//	remap(rightCamera->frame,rightCamera->calibratedframe, rmap[1][0], rmap[1][1], INTER_LINEAR);
}


int main()
{
  int patternchoice;
  int choice;
  cameraCalibrate *rightCamera = new  cameraCalibrate(2);
  cameraCalibrate *leftCamera = new  cameraCalibrate(1);
  stereoCameraCalibrate *stereoCamera = new  stereoCameraCalibrate(leftCamera,rightCamera);
  Size patternsize;
  //enclosed corners horizontally on chessboard
  int board_corners_w;
  //enclosed corners vertically on chessboard
  int board_corners_h;
  float squareSize; //one grid
  Size circlepatternsize;
  circlepatternsize=Size(4,11);
  //enclosed centers horizontally
  int board_centers_w;
  //enclosed centers vertically
  int board_centers_h;
  float circleSize; //radius
  int snapshot=0;
 //KeyPressed is used to continue in while loop
  int KeyPressed=255;   //255 is an example
  cout <<"Camera calibration begins!!!!" << endl;
  cout<<"*************************************************"<<endl;
  cout<<"Vision rocks!!!!!!!!!!!!!!!!"<<endl;
  cout<<"Choose desirable pattern"<<endl;
  cout<<"Press 1 for square pattern"<<endl;
  cout<<"Press 2 for circle pattern"<<endl;
  cin>>patternchoice;
    if(patternchoice==1)
    {
        cout<<"Give the number of enclosed corners horizontally"<<endl;
        cin>>board_corners_w;
        cout<<"Give the number of enclosed corners vertically"<<endl;
        cin>>board_corners_h;
        cout<<"Give squaresize"<<endl;
        cin>>squareSize;
        patternsize=Size(board_corners_w,board_corners_h);
    }
    else
    {
        cout<<"Give the number of enclosed centers horizontally"<<endl;
        cin>>board_centers_w;
        cout<<"Give the number of enclosed centers vertically"<<endl;
        cin>>board_centers_h;
        cout<<"Give radius"<<endl;
        cin>>circleSize;
        circlepatternsize=Size(board_centers_w,board_centers_h);
    }
    cout<<"Choose camera for calibration"<<endl;
    cout<<"If you want to calibrate left camera press 1"<<endl;
    cout<<"If you want to calibrate right camera press 2"<<endl;
    cout<<"If you want to calibrate stereo camera press 3"<<endl;
    cin>>choice;
    while(1)
    {
        while(choice==1)
        {
            cout<<"Left camera's calibration begins"<<endl;
            while(1)
            {
               // cout<<"If you wish tou take a new frame press 0 else press 1"<<endl;
                //cin>>snapshot;
                leftCamera->createSnapshot();
                leftCamera->detectChessboard(patternchoice,patternsize,circlepatternsize);
                leftCamera->refreshWindow();
               // cvWaitKey(10);
                KeyPressed=cvWaitKey(10) & 255;
                //different choices
                if(KeyPressed==27) //KeyPressed==esc
                {
                    break;
                }
                if (KeyPressed==99) //KeyPressed==c
                {
                    leftCamera->saveSnapshot();
                }
                if (KeyPressed==112) //KeyPressed==p
                {
                    cout<<"Set points is called"<<endl;
                    leftCamera->setPoints(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
                }
                if(KeyPressed==115) //KeyPressed==s
                {
                    leftCamera->calibratefunction();
                }
                leftCamera->refreshWindow();
            }
             cout<<"If you want to change camera: press 2 for right and 3 for stereo"<<endl;
             cin>>choice;
        }
        while(choice==2)
        {
           cout<<"Right camera's calibration begins"<<endl;
            while(1)
            {
               // cout<<"If you wish tou take a new frame press 0 else press 1"<<endl;
                //cin>>snapshot;
                rightCamera->createSnapshot();
                rightCamera->detectChessboard(patternchoice,patternsize,circlepatternsize);
                rightCamera->refreshWindow();
               // cvWaitKey(10);
                KeyPressed=cvWaitKey(10) & 255;
                //different choices
                if(KeyPressed==27) //KeyPressed==esc
                {
                    break;
                }
                if (KeyPressed==99) //KeyPressed==c
                {
                   rightCamera->saveSnapshot();
                }
                if (KeyPressed==112) //KeyPressed==p
                {
                    cout<<"Set points is called"<<endl;
                    rightCamera->setPoints(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
                }
                if(KeyPressed==115) //KeyPressed==s
                {
                     rightCamera->calibratefunction();
                }
                 rightCamera->refreshWindow();
            }
             cout<<"If you want to change camera: press 2 for right and 3 for stereo"<<endl;
             cin>>choice;
        }
        while(choice==3)
        {
             while(1)
            {
                stereoCamera->stereoCreateSnapshot();
                stereoCamera->stereodetectChessboard(patternchoice,patternsize,circlepatternsize);
                stereoCamera->refreshWindow();
               // cvWaitKey(10);
                KeyPressed=cvWaitKey(10) & 255;
                //different choices
                if(KeyPressed==27) //KeyPressed==esc
                {
                    break;
                }
                if (KeyPressed==99) //KeyPressed==c
                {
                    stereoCamera->saveSnapshot();
                }
                if (KeyPressed==112) //KeyPressed==p
                {
                    cout<<"Set points is called"<<endl;
                    stereoCamera->stereosetPoints(patternchoice,patternsize,circlepatternsize,squareSize,circleSize);
                }
                if(KeyPressed==115) //KeyPressed==s
                {
                     stereoCamera->stereoCalibratefunction();
                }
                  stereoCamera->refreshWindow();
            }
             cout<<"If you want to change camera: press 2 for right and 3 for stereo"<<endl;
             cin>>choice;
        }
            stereoCamera->savedata();
        }
    waitKey(0);
    delete(rightCamera);
    delete(leftCamera);
    delete(stereoCamera);
    return 0;
}
