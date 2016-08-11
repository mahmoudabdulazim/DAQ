#include <stdint.h>
#include <unistd.h>
#include "Monocular-VO/Mono-VO.h"
#include <math.h>

#define ONE_MS 1000
#define TEN_MS 100

// Declaration & Definitions of some Helper functions

void RotationMat2Angles(Mat R,double (&EulerZYX)[3])
{
        EulerZYX[2] = atan2(R.at<double>(2,1), R.at<double> (2,2))*180/M_PI;
        EulerZYX[1] = asin(R.at<double>(2,0))*180/M_PI;
        EulerZYX[0] = -atan2(R.at<double>(1,0), R.at<double>(0,0))*180/M_PI;
}

void RotationMat2ARates(Mat R, Mat R_dot, double (&AngularVel)[3])
{

}


// Definitions of Class member functions
bool MonoVOdometry::Initialize(char* argv[])
{
	// Camera Information Extraction
	if ((sizeof(argv)/__SIZEOF_POINTER__) == 2)
	{
		if (!(RPiCamInfo.ReadCharacteristics(string(argv[1]))))
        	{
            		std::cout << "Reading Characteristics Data failed" << std::endl;
            		return 0;
        	}
    	}
    	else if ((sizeof(argv)/__SIZEOF_POINTER__) > 2)
    	{
        	if (!(RPiCamInfo.ReadCalibrationData(string(argv[1]))))
        	{
        		std::cout << "Reading Calibration Data failed" << std::endl;
            		return 0;
        	}
        	RPiCamInfo.SaveCharacteristics(string(argv[2]));
    	}
    	else
    	{
        	std::cout << "Couldn't Open Camera Information File, Node will exit" << endl;
        	return 0;
    	}

    	// Camera Configuration and Initialization

    	RPiCam.set(CV_CAP_PROP_FRAME_WIDTH,RPiCamInfo.ImageWidth);
	RPiCam.set(CV_CAP_PROP_FRAME_HEIGHT,RPiCamInfo.ImageHeight);

    	if (!RPiCam.open())
    	{
        	std::cout << "Opening Camera failed" << std::endl;
        	return 0;
    	}
    	else
    	{
        	std::cout << "Waiting for Camera Stabilization" << std::endl;
        	usleep(100*TEN_MS);
    	}


	// Capturing the first frame and initializing the computational cycle
	RPiCam.grab();
	RPiCam.retrieve(prev_frame);

	feat_xt.detectAndCompute(prev_frame,noArray(),prev_KPs,prev_descrs);
	R_f = Mat::eye(3,3,CV_64F);
	t_f = Mat::zeros(3,1,CV_64F);
}

void MonoVOdometry::captureFrame()
{
	RPiCam.grab();
	curr_frame.copyTo(prev_frame);
    	RPiCam.retrieve(curr_frame);
}

void MonoVOdometry::captureFrameUndistorted()
{
    	captureFrame();
    	undistort(curr_frame,curr_frame,RPiCamInfo.IntrinsicParameters,RPiCamInfo.DistortionCoefficients);
}

void MonoVOdometry::computeFeatures()
{
	feat_xt.detectAndCompute(curr_frame,noArray(),curr_KPs,curr_descrs);
}

void MonoVOdometry::matchFeatures()
{
	if (!(curr_descrs.empty()) && !(prev_descrs.empty()))
	{
		matcher.match(curr_descrs, prev_descrs, matches);
	}
	else
	{
		cout << "Train or Query descriptors are empty" << endl;
	}
}

void MonoVOdometry::extractGoodCorrespondencies()
{
	double min_dist;
	  //-- Quick calculation of max and min distances between keypoints
  	for ( int i = 0; i < curr_descrs.rows; ++i )
  	{
		double dist = matches[i].distance;
  		if( dist < min_dist ) min_dist = dist;
	}

	for (int i = 0; i < curr_descrs.rows; ++i)
	{
		if (matches[i].distance <= max(2*min_dist,0.02))
		{
			matched_prev_KPs.push_back(prev_KPs[matches[i].trainIdx]);
			matched_curr_KPs.push_back(curr_KPs[matches[i].queryIdx]);
		}
	}
	KeyPoint::convert(matched_prev_KPs, prev_points);
	KeyPoint::convert(matched_curr_KPs, curr_points);
}

void MonoVOdometry::computeVOdometry()
{

	Mat E = findEssentialMat(prev_points,curr_points,1,Point2d(0,0),RANSAC,0.999,1);

	int inliers = recoverPose(E,matched_curr_KPs, matched_prev_KPs,R,t,1,Point2d(0,0));

	if (inliers < 8)
        {
        	if (inliers == 0)
		{
                	cout << "Camera didn't move" << endl;
			Vel[0] = 0.0;
			Vel[1] = 0.0;
			Vel[2] = 0.0;
		}
                else
                	cout << "Insufficient cheirality test inliers" << endl;
	}
        else
        {
                R_dot = R-R_prev;

                t_f = t_f + 1*(R_f*t);
                R_f = R*R_f;

		RotationMat2Angles(R_f,this->EulerZYX);
		RotationMat2ARates(R_f,R_dot,this->AngularVel);

                Vel[0]    = t.at<double>(0);
                Vel[1]    = t.at<double>(1);
                Vel[2]    = t.at<double>(2);

		Pos[0]    = t_f.at<double>(0);
		Pos[1]    = t_f.at<double>(1);
		Pos[2]    = t_f.at<double>(2);
	}

	curr_frame.copyTo(prev_frame);
	R.copyTo(R_prev);
	prev_descrs = curr_descrs;
	prev_KPs    = curr_KPs;
}

void MonoVOdometry::Release()
{

}


void RotationMat2Angles(Mat R_f,double (&EulerZYX)[3])
{

}
void RotationMat2ARates(Mat R_f, Mat R_dot, double (&AngularVel)[3])
{

}
