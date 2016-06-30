/*

The MIT License

Copyright (c) 2015 Avi Singh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <raspicam/raspicam_cv.h>
#include "mono-vo/vo_features.h"

using namespace cv;
using namespace std;

#define MIN_NUM_FEAT 2000

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)
{
	/* This function deduces the absolute scale from IMU readings. */
	return 0;
}


int main( int argc, char** argv )
{
	ros::init(argc,argv,"RpiCamera");
	ros::NodeHandle RPiC;
	raspicam::RaspiCam_Cv RPiCam;

  	Mat img_old, img_new, R_f = Mat::eye(3, 3, CV_64F), t_f = Mat::zeros(3,1, CV_64F); //the final rotation and tranlation vectors containing the 
	Mat E, R, t, mask;

	vector<Point2f> points_old, points_new;        //vectors to store the coordinates of the feature points

	double scale = 1.00;

	RPiCam.set(CV_CAP_PROP_FRAME_WIDTH,320*1.5);
	RPiCam.set(CV_CAP_PROP_FRAME_HEIGHT,240*1.5);

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
	
	featureDetection(img_old, points_old);        //detect features in img_old

	vector<uchar> status;

	while(ros::ok())
	{
		RPiCam.grab();
		RPiCam.retrieve(img_new);
		cvtColor(img_new,img_new,COLOR_BGR2GRAY);
		
		featureTracking(img_old,img_new,points_old,points_new, status); //track those features to img_new

		double focal = 718.8560;
		cv::Point2d pp(607.1928, 185.2157);
		//recovering the pose and the essential matrix
		if ((points_new.size() > 5) && (points_old.size() > 5))
		{
			E = findEssentialMat(points_new, points_old, focal, pp, RANSAC, 0.999, 1.0, mask);
			recoverPose(E, points_new, points_old, R, t, focal, pp, mask);
		
			t_f = t_f + scale*(R_f*t);
			R_f = R*R_f;
		}
		else
		{
			cout << "Number of features went below accepted value" << endl;
		}
		if (points_old.size() < MIN_NUM_FEAT)	
		{
	 		featureDetection(img_old, points_old);
			featureTracking(img_old,img_new,points_old,points_new, status);
		}
		
		img_old = img_new.clone();
		points_old = points_new;
		waitKey(1);

		cout << "R Matrix is" << R_f << endl;
		cout << "t Vector is" << t_f << endl;
		imshow("Preview Image",img_old);

	}
	RPiCam.release();
	return 0;
}
