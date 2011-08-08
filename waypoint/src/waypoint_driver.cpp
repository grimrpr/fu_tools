#include "ros/ros.h"
#include <ros/package.h>
#include "waypoint/GetTracks.h"
#include "waypoint/GetRoutes.h"
#include "waypoint/GenerateTrack.h"
#include "waypoint/MarkWaypoint.h"
#include "waypoint/PlayBackTrack.h"
#include "waypoint/RunTrack.h"
#include "waypoint/DeleteTrack.h"
#include "waypoint/DeleteRoute.h"
#include <sstream>
#include <fstream>
#include <stdio.h>

std::string storagePath;

bool fileExists(const std::string &fileName)
{
  std::fstream f;

  f.open(fileName.c_str(),std::ios::in);
  if( f.is_open() )
  {
    f.close();
    return true;
  }
  f.close();
  return false;
}

std::string formatTime(const unsigned int sec)
{
  
  std::stringstream t;

  /*
  unsigned int min    = sec / 60;
  unsigned int hours  = min / 60;
  unsigned int days   = hours / 24;
  */

  return t.str();
  
}
bool getTracks(waypoint::GetTracks::Request &req, waypoint::GetTracks::Response &res)
{
  ROS_INFO("getTracks called");

	std::vector<waypoint::Track> vec;
	waypoint::Track track;
	track.name="Hans";
  track.time=ros::Time::now();
	vec.push_back(track);
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

  std::stringstream fileName;
  fileName << req.track.time.toSec() << "_t_" << req.track.name;
  ROS_INFO(fileName.str().c_str());

  if(fileExists(fileName.str()))
  {
    remove(fileName.str().c_str());
    ROS_INFO("track removed");
    res.successfull = true;
    return true;
  }

  ROS_INFO("track not found");
  res.successfull = false;
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

  std::stringstream path;
  path << ros::package::getPath("waypoint") << "/storage/";
  storagePath = path.str();
  
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
