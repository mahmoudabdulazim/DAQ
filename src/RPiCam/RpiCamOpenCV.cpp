#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <raspicam/raspicam_cv.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;
using namespace raspicam;

int main(int argc,char **argv)
{
	ros::init(argc,argv,"RpiCamera");
	ros::NodeHandle RPiC;
	RaspiCam_Cv RPiCam;
	Mat image;
	if (!RPiCam.open())
	{
		cerr << "Error encountered while trying to open camera" << endl;
		return -1;
	}

	while(ros::ok())
	{
		RPiCam.grab();
		RPiCam.retrieve(image);
		imshow("Image Preview",image);
		waitKey(1);
		ros::spinOnce();
	}

	cout << "Stoping Camera ... " <<  endl;
	RPiCam.release();
	return 0;
}
