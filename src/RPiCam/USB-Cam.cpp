#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace  cv;

int main()
{
	VideoCapture camera;
	camera.open(0);
	Mat frame;
	while(camera.isOpened())
	{
		camera.read(frame);
		imshow("Preview",frame);
		waitKey(1);
	}
}
