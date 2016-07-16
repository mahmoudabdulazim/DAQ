#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <raspicam/raspicam_cv.h>
#include "mono-vo/vo_features.h"
#include <sys/time.h>
#include "daq/SixDoF.h"
#include "mono-vo/CameraInfo.h"
#include <string.h>

using namespace cv;
using namespace std;

#define MIN_NUM_FEAT 2000

int main( int argc, char** argv )
{
	ros::init(argc,argv,"RpiCamera");
	ros::NodeHandle RPiC;
	ros::Publisher CAM = RPiC.advertise<daq::SixDoF>("VO_Info",10);

	raspicam::RaspiCam_Cv RPiCam;

	timespec tic,toc;
	double dt;
  	Mat img_old, img_new,img_f, R_f = Mat::eye(3, 3, CV_64F), t_f = Mat::zeros(3,1, CV_64F),Mask,R_prev,Rdot,R,t,E;
	CameraInfo RPiCameraInfo;

	if (argc > 2)
	{
		RPiCameraInfo.ReadCalibrationData(string(argv[1]));
		RPiCameraInfo.SaveCharacteristics(string(argv[2]));
	}
	else if (argc == 2)
	{
		RPiCameraInfo.ReadCharacteristics(argv[1]);
	}
	else
	{
		cout << "Couldn't Open Camera Information File, Node will exit" << endl;
		return -1;
	}

	vector<Point2f> points_old, points_new;        //vectors to store the coordinates of the feature points

	double scale = 1.00;

	RPiCam.set(CV_CAP_PROP_FRAME_WIDTH,RPiCameraInfo.ImageWidth);
	RPiCam.set(CV_CAP_PROP_FRAME_HEIGHT,RPiCameraInfo.ImageHeight);

	if (!RPiCam.open())
	{
		cerr << "Error encountered while trying to open camera" << endl;
		return -1;
	}

  	RPiCam.grab();
	RPiCam.retrieve(img_old);

  	if ( !img_old.data)
	{
		std::cout<< " --(!) Error reading images " << std::endl;
		return -1;
	}

	cvtColor(img_old, img_old, COLOR_BGR2GRAY);

	featureDetection(img_old, points_old,img_f);        //detect features in img_old

	vector<uchar> status;

	while(ros::ok())
	{
		clock_gettime(CLOCK_REALTIME, &tic);
		RPiCam.grab();
		RPiCam.retrieve(img_new);
		cvtColor(img_new,img_new,COLOR_BGR2GRAY);

		featureTracking(img_old,img_new,points_old,points_new, status); //track those features to img_new

		//Recovering Pose
		if ((points_new.size() > 5) && (points_old.size() > 5))
		{
			R.copyTo(R_prev);
			E = findEssentialMat(points_old,points_new,RPiCameraInfo.FocalLength,RPiCameraInfo.PrincipalPoint,RANSAC,0.999,1,Mask);
			recoverPose(E, points_new, points_old, R, t, RPiCameraInfo.FocalLength, RPiCameraInfo.PrincipalPoint,Mask);

			Rdot = R-R_prev;

			t_f = t_f + scale*(R_f*t);
			R_f = R*R_f;
		}
		else
		{
			cout << "Number of features went below accepted value" << endl;
		}
		if (points_old.size() < MIN_NUM_FEAT)
		{
	 		featureDetection(img_old, points_old,img_f);
			featureTracking(img_old,img_new,points_old,points_new, status);
		}

		img_old = img_new.clone();
		points_old = points_new;
		imshow("Preview",img_f);
		waitKey(1);

		daq::SixDoF R_t;

		R_t.Px    = t_f.at<float>(0);
		R_t.Py    = t_f.at<float>(1);
		R_t.Pz    = t_f.at<float>(2);
		R_t.vx    = t.at<float>(0);
		R_t.vy    = t.at<float>(1);
		R_t.vz    = t.at<float>(2);
		R_t.Yaw   = 0;
		R_t.Pitch = 0;
		R_t.Roll  = 0;
		R_t.q     = 0;
		R_t.p     = 0;
		R_t.r     = 0;

		CAM.publish(R_t);

		clock_gettime(CLOCK_REALTIME, &toc);
		dt = abs((double) toc.tv_nsec - (double) tic.tv_nsec);
		printf("Sample Time is : %f \n",dt/1000000000);
	}
	RPiCam.release();
	return 0;
}

void RotationMat2Angles(const Mat& R,float (&ypr)[3])
{

}

void RotationMat2Rates(const Mat& R,const Mat& Rdot,float (&qpr)[3])
{

}
