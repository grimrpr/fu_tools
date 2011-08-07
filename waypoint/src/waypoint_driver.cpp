#include "ros/ros.h"
#include "waypoint/GetTracks.h"
#include "waypoint/GetRoutes.h"
#include "waypoint/GenerateTrack.h"
#include "waypoint/MarkWaypoint.h"
#include "waypoint/PlayBackTrack.h"
#include "waypoint/RunTrack.h"
#include "waypoint/DeleteTrack.h"
#include "waypoint/DeleteRoute.h"
#include <sstream>

bool getTracks(waypoint::GetTracks::Request &req, waypoint::GetTracks::Response &res)
{
  ROS_INFO("getTracks called");

	std::vector<waypoint::Track> vec;
	waypoint::Track track;
	track.name="Hans";
	track.routeCount=2;
	vec.push_back(track);

	res.tracks=vec;
	return true;
}

bool generateTrack(waypoint::GenerateTrack::Request &req, waypoint::GenerateTrack::Response &res)
{
	ROS_INFO("generateTrack called");
	return true;
}

bool getRoutes(waypoint::GetRoutes::Request &req, waypoint::GetRoutes::Response &res)
{
	ROS_INFO("getRoutes called");
	return true;
}

bool markWaypoint(waypoint::MarkWaypoint::Request &req, waypoint::MarkWaypoint::Response &res)
{
	ROS_INFO("markWaypoint called");
	return true;
}

bool playBackTrack(waypoint::PlayBackTrack::Request &req, waypoint::PlayBackTrack::Response &res)
{
	ROS_INFO("playBackTrack called");
	return true;
}

bool runTrack(waypoint::RunTrack::Request &req, waypoint::RunTrack::Response &res)
{
  ROS_INFO("runTrack called");
  return true;
}

bool deleteTrack(waypoint::DeleteTrack::Request &req, waypoint::DeleteTrack::Response &res)
{
	ROS_INFO("deleteTrack called");
	return true;
}

bool deleteRoute(waypoint::DeleteRoute::Request &req, waypoint::DeleteRoute::Response &res)
{
	ROS_INFO("deleteRoute called");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_driver");
	ros::NodeHandle n;

	ros::ServiceServer srvGetTracks     	= n.advertiseService("GetTracks", getTracks);
	ros::ServiceServer srvGetRoutes     	= n.advertiseService("GetRoutes", getRoutes);
	ros::ServiceServer srvGenerateTrack	  = n.advertiseService("GenerateTrack", generateTrack);
	ros::ServiceServer srvMarkWaypoint  	= n.advertiseService("MarkWaypoint", markWaypoint);
	ros::ServiceServer srvRunTrack      	= n.advertiseService("RunTrack", runTrack);
	ros::ServiceServer srvPlayBackTrack		= n.advertiseService("PlayBackTrack", playBackTrack);
	ros::ServiceServer srvDeleteTrack	    = n.advertiseService("DeleteTrack", deleteTrack);
	ros::ServiceServer srvDeleteRoute	    = n.advertiseService("DeleteRoute", deleteRoute);

	ROS_INFO("Services initialized");

	ros::spin();

	/*
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ROS_INFO("Running");
		ROS_DEBUG("DRUNNING");
		ros::spinOnce();
		loop_rate.sleep();
	}
	*/

	return 0;
}
