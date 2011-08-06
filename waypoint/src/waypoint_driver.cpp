#include "ros/ros.h"
#include "waypoint/GetTracks.h"

#include <sstream>

bool getTracks(waypoint::GetTracks::Request &req)
{
	ROS_INFO(req.name);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_driver");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("GetTracks", getTracks);
	ROS_INFO("Service GetTracks initialized");
	ros::spin();

	/*
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
	}
	*/

	return 0;
}
