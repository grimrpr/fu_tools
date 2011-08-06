#include "ros/ros.h"
#include "waypoint/GetTracks.h"
#include "waypoint/RecordTrack.h"
#include "waypoint/MarkWaypoint.h"
#include "waypoint/PlayTrack.h"
#include "waypoint/DeleteTrack.h"
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

bool recordTrack(waypoint::RecordTrack::Request &req, waypoint::RecordTrack::Response &res)
{
	ROS_INFO("recordTrack called");
	return true;
}

bool markWaypoint(waypoint::MarkWaypoint::Request &req, waypoint::MarkWaypoint::Response &res)
{
	ROS_INFO("markWaypoint called");
	return true;
}

bool playTrack(waypoint::PlayTrack::Request &req, waypoint::PlayTrack::Response &res)
{
	ROS_INFO("playTrack called");
	return true;
}

bool deleteTrack(waypoint::DeleteTrack::Request &req, waypoint::DeleteTrack::Response &res)
{
	ROS_INFO("deleteTrack called");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_driver");
	ros::NodeHandle n;

	ros::ServiceServer srvGetTracks 	= n.advertiseService("GetTracks", getTracks);
	ros::ServiceServer srvRecordTrack	= n.advertiseService("RecordTrack", recordTrack);
	ros::ServiceServer srvMarkWaypoint	= n.advertiseService("MarkWaypoint", markWaypoint);
	ros::ServiceServer srvPlayTrack		= n.advertiseService("PlayTrack", playTrack);
	ros::ServiceServer srvDeleteTrack	= n.advertiseService("DeleteTrack", deleteTrack);

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
