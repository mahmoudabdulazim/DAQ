#include <sys/time.h>
#include <iostream>
#include <vector>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <raspicam/raspicam_cv.h>

#include <daq/Quaternion.h>
#include <Monocular-VO/CameraInfo.h>

using namespace std;
using namespace cv;
using namespace raspicam;

void RotationMat2Angles(Mat R,double (&EulerZYX)[3]);
void RotationMat2ARates(Mat R, Mat R_dot, double (&AngularVel)[3]);

class MonoVOdometry
{
    public:
// Attributes:

//	Camera handle and Camera information container
        RaspiCam_Cv RPiCam;
        CameraInfo RPiCamInfo;

//	Internal containers of images
        Mat prev_frame, curr_frame;

//	Keypoints in consequetive frames
        vector<KeyPoint> prev_KPs,curr_KPs;
        vector<KeyPoint> matched_prev_KPs,matched_curr_KPs;

//	Point2f vectors for float operations
	vector<Point2f> prev_points, curr_points;

//	Descriptors matrices
	Mat prev_descrs, curr_descrs;

//	Output Rotation and Translation
	Mat R,t,t_f,R_f,R_dot,R_prev;

//	Output 6DoF vectors
        double Pos[3],Vel[3],EulerZYX[3],AngularVel[3];

//	Time statistics
        timespec tic, toc;

//	Feature extractor and descriptor
	xfeatures2d::StarDetector feat_xt;

//	Feature Matcher
	FlannBasedMatcher matcher = FlannBasedMatcher(makePtr<flann::KMeansIndexParams>());

//	Vector of matches
	vector<DMatch> matches;


//-------------------- Methods------------------\\

//	Initialization function:
/*	This function serves  to do the following:
		1) Read Camera information from file
		2) Open and set camera parameters
*/
        bool Initialize(char* argv[]);

//	Function as a wrapper around RPi-Camera API for capturing images
        void captureFrame();

//	Same as above, but further adds undoistortions using the information provided by CameraInfo container
        void captureFrameUndistorted();

//	Extract and describe features
        void computeFeatures();

//	Match the descriptors
        void matchFeatures();

//	Extracting Good matches
	void extractGoodCorrespondencies();

//	Use successful matches in computing odometry information
        void computeVOdometry();

//	Clean up
        void Release();
};
