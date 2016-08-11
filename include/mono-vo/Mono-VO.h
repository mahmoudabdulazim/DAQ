#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>

using namespace std;
using namespace  cv;


class VOdom
{
	public:
		Mat prevImg, currImg;
		Mat r, t, R, T;
}