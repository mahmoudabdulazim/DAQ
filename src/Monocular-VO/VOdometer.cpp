#include <ros/ros.h>
#include <Monocular-VO/Mono-VO.h>
#include <daq/SixDoF.h>

using namespace ros;

int main(int argc,char* argv[])
{
	init(argc, argv,"VOdometer");

	NodeHandle VOdometer;
	MoboVodometry RPiVOdom;

	Publisher VO_broadcaster = advertise<daq::SixDof>("vodometry",1);

	daq::SixDof vodom;

	RPiVOdom.Initialize();
	while(ok())
	{
		RPiVOdom.captureFrameUndistorted();
		RPiVOdom.computeFeatures();
		RPiVOdom.matchFeatures();
		RPiVOdom.extractGoodCorrespondencies();
		RPiVOdom.computeVOdometry();


		vodom.Px = RPiVOdom.Pos[0];
		vodom.Py = RPiVOdom.Pos[1];
		vodom.Pz = RPiVOdom.Pos[2];
		vodom.vx = RPiVOdom.Vel[0];
		vodom.vy = RPiVOdom.Vel[1];
		vodom.vz = RPiVOdom.Vel[2];
	}
}
