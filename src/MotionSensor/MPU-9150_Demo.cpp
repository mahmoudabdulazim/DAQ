#include "ros/ros.h"
#include "MotionSensor/MPU-9150.h"
#include <daq/SixDoF.h>

using namespace std;
using namespace ros;

int main(int argc,char* argv[])
{
	init(argc,argv,"IMU_Demo");
	MPU9150 IMU;
	NodeHandle IMU_Node;
	Rate loop_rate(100);

	ros::Publisher pub=IMU_Node.advertise<daq::SixDoF>("IMU",10);

	IMU.Initialize();

	while(ok())
	{
		IMU.Read();
		daq::SixDoF IMU6DoF;
		IMU6DoF.Px = 0;
		IMU6DoF.Py = 0;
		IMU6DoF.Pz = 0;
		IMU6DoF.vx = 0;
		IMU6DoF.vy = 0;
		IMU6DoF.vz = 0;
		IMU6DoF.Yaw=IMU.ypr[0];
		IMU6DoF.Pitch= IMU.ypr[1];
		IMU6DoF.Roll =IMU.ypr[2];
		IMU6DoF.q =IMU.gyro[0];
		IMU6DoF.p = IMU.gyro[1];
		IMU6DoF.r = IMU.gyro[2];
		pub.publish(IMU6DoF);
		
		ROS_INFO("Yaw is: %f, Pitch is: %f, Roll is: %f || Compass is: %f %f %f || Acceleration is: %f %f %f",IMU.ypr[0],IMU.ypr[1],IMU.ypr[2],IMU.compass[0],IMU.compass[1],IMU.compass[2],IMU.accel[0],IMU.accel[1],IMU.accel[2]);
	
	        
		spinOnce();
		loop_rate.sleep();
	}
}
