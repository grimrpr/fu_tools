#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/PoseStamped.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "waypoint/GetTracks.h"
#include "waypoint/GetRoutes.h"
#include "waypoint/StartNewTrack.h"
#include "waypoint/SaveNewTrack.h"
#include "waypoint/MarkWaypoint.h"
#include "waypoint/PlayBack.h"
#include "waypoint/RunTrack.h"
#include "waypoint/DeleteTrack.h"
#include "waypoint/DeleteRoute.h"
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <boost/foreach.hpp>

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *server;
std::string storagePath;
waypoint::Track recordedTrack;
std::deque<geometry_msgs::PoseStamped> recordedTrackPoses;
std::deque<geometry_msgs::PoseStamped> recordedRoutePoses;
geometry_msgs::PoseStamped currentPose;
bool doRecordNewTrack = false;
bool doMarkWaypoint = false;
bool doRunTrack = false;
bool doSendNextGoal = false;

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

std::vector<std::string> getStorageFiles(char type, std::string name)
{

  std::vector<std::string> entries;

  struct dirent *entry;
  DIR *directory = opendir(storagePath.c_str());

  if(directory == 0)
  {
    ROS_INFO("directory == 0");
    return std::vector<std::string>();
  }

  while((entry=readdir(directory)))
  {
    std::string str(entry->d_name);
    if(str == "." || str == "..")
    {
    }
    else
    {  
      char t;
      char n[1024];
      sscanf(str.c_str(),"%*[0-9]_%*[0-9]_%c_%[a-z-A-Z-0-9]", &t, n);
      if(type != NULL && type != t)
      {
        continue;
      }
      if(!name.empty() && std::string(n) != name)
      {
        continue;
      }
      entries.push_back(str);
    }
  }

  closedir(directory);

  return entries;
}

bool getTracks(waypoint::GetTracks::Request &req, waypoint::GetTracks::Response &res)
{
  ROS_INFO("getTracks called");

	std::vector<waypoint::Track> tracks;

  std::vector<std::string> entries = getStorageFiles('T', std::string(""));

  for(unsigned int i = 0; i < entries.size(); ++i)
  {
    char timeDate[8];
    char timeClock[6];
    char name[1024];
    sscanf(entries[i].c_str(),"%[0-9]_%[0-9]_%*c_%[a-z-A-Z-0-9]", timeDate, timeClock, name);
    waypoint::Track track;
    track.name = std::string(name);
    track.time = std::string(timeDate) + std::string("_") + std::string(timeClock);    
    tracks.push_back(track);
  }

	res.tracks=tracks;

	return true;
}

bool getRoutes(waypoint::GetRoutes::Request &req, waypoint::GetRoutes::Response &res)
{
	ROS_INFO("getRoutes called");
	
  std::vector<waypoint::Route> routes;

  std::vector<std::string> entries = getStorageFiles('R', req.track.name);

  for(unsigned int i = 0; i < entries.size(); ++i)
  {
    char timeDate[8];
    char timeClock[6];
    char name[1024];
    sscanf(entries[i].c_str(),"%[0-9]_%[0-9]_%*c_%[a-z-A-Z-0-9]", timeDate, timeClock, name);
    waypoint::Route route;
    route.name = std::string(name);
    route.time = std::string(timeDate) + std::string("_") + std::string(timeClock);    
    routes.push_back(route);
  }

	res.routes=routes;

	return true;
}

std::string getCurrentTimeString()
{
  // build time string like that: 19881030_082259 (yyyyMMdd_hhmmss)
  time_t time;
  tm *now;
  time = std::time(0);
  now = localtime(&time);
  std::stringstream currentTime;

  currentTime << (now->tm_year+1900);

  if(now->tm_mon < 9)     {currentTime << "0";}
  currentTime << (now->tm_mon + 1);

  if(now->tm_mday < 10)   {currentTime << "0";}
  currentTime << now->tm_mday;

  currentTime << "_";
  
  if(now->tm_hour < 10)   {currentTime << "0";}
  currentTime << now->tm_hour;
  
  if(now->tm_min < 10)    {currentTime << "0";}
  currentTime << now->tm_min;

  if(now->tm_sec < 10)    {currentTime << "0";}
  currentTime << now->tm_sec;

  return currentTime.str();
}

bool startNewTrack(waypoint::StartNewTrack::Request &req, waypoint::StartNewTrack::Response &res)
{
	ROS_INFO("startNewTrack called");

  if(doRecordNewTrack)
  {
    ROS_INFO("Only one track can be recorded at a time");
    return true;
  }

  if(getStorageFiles('T', req.track.name).size() > 0)
  {
    ROS_INFO("Track name already exists");
    return true;
  }

  recordedTrack = req.track;

  recordedTrack.time = getCurrentTimeString();
  doRecordNewTrack = true;

  res.track = recordedTrack;
  res.successful = true;
	return true;
}

bool saveNewTrack(waypoint::SaveNewTrack::Request &req, waypoint::SaveNewTrack::Response &res)
{
	ROS_INFO("saveNewTrack called");

  if(recordedTrack.name != req.track.name || recordedTrack.time != req.track.time)
  {
    ROS_INFO("Given track is not the one that is currently recording");
    return true;
  }

  // Stop recording
  doRecordNewTrack = false;
  doMarkWaypoint = false;

  if(recordedTrackPoses.empty())
  {
    ROS_INFO("No waypoints were marked, saving anyway");
  }

  // Persist track file
  rosbag::Bag bag;
  std::stringstream filename;
  filename << storagePath << recordedTrack.time << "_T_" << recordedTrack.name << ".bag";
  bag.open(filename.str().c_str(), rosbag::bagmode::Write);
  for(unsigned int i = 0; i < recordedTrackPoses.size(); ++i)
  {
    bag.write("track", recordedTrackPoses[i].header.stamp, recordedTrackPoses[i]);
  }
  bag.close(); 

  // Clean recorded poses
  recordedTrackPoses.clear();
  
  res.successful = true;
	return true;
}

bool markWaypoint(waypoint::MarkWaypoint::Request &req, waypoint::MarkWaypoint::Response &res)
{
	ROS_INFO("markWaypoint called");

  if(doRecordNewTrack)
  {
    doMarkWaypoint = true;
    res.successful = true;
  }
  else
  {
    res.successful = false;
  }

	return true;
}

bool playBack(waypoint::PlayBack::Request &req, waypoint::PlayBack::Response &res)
{
	ROS_INFO("playBackTrack called, BUT NOT IMPLEMENTED!");
	return true;
}

bool runTrack(waypoint::RunTrack::Request &req, waypoint::RunTrack::Response &res)
{ 
  ROS_INFO("runTrack called");

  if(doRunTrack)
  {
    return true;
  }

  // Load track from bag file
  recordedTrackPoses.clear();
  std::stringstream filename;
  filename << storagePath << req.track.time << "_T_" << req.track.name << ".bag";
  rosbag::Bag bag(filename.str().c_str());
  rosbag::View view(bag, rosbag::TopicQuery("track"));
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    geometry_msgs::PoseStampedConstPtr pose = m.instantiate<geometry_msgs::PoseStamped>();
    if (pose != NULL)
      recordedTrackPoses.push_back(*pose);
  }
  bag.close();

  // Start sending loaded waypoints
  doRunTrack = true;
  doSendNextGoal = true;

  res.successful = true;
  return true;
}

bool deleteTrack(waypoint::DeleteTrack::Request &req, waypoint::DeleteTrack::Response &res)
{

	ROS_INFO("deleteTrack called");

  std::stringstream fileName;
  fileName << storagePath << req.track.time << "_T_" << req.track.name << ".bag";

  if(fileExists(fileName.str()))
  {
    remove(fileName.str().c_str());

    std::vector<std::string> entries = getStorageFiles('R', req.track.name);
    for(unsigned int i = 0; i < entries.size(); ++i)
    {
      std::stringstream fileName;
      fileName << storagePath << entries[i];
      remove(fileName.str().c_str());
    }

    res.successful = true;
    return true;
  }

  ROS_INFO("track not found");
  res.successful = false;
	return true;

}

bool deleteRoute(waypoint::DeleteRoute::Request &req, waypoint::DeleteRoute::Response &res)
{
	ROS_INFO("deleteRoute called");

  std::vector<std::string> entries = getStorageFiles('R', req.route.name);
  for(unsigned int i = 0; i < entries.size(); ++i)
  {
    char timeDate[8];
    char timeClock[6];
    sscanf(entries[i].c_str(),"%[0-9]_%[0-9]_%*c_%*[a-z-A-Z-0-9]", timeDate, timeClock);
    if((std::string(timeDate) + std::string("_") + std::string(timeClock)) == req.route.time)
    {
      std::stringstream fileName;
      fileName << storagePath << entries[i];
      remove(fileName.str().c_str());
    }
  }

  res.successful = true;
	return true;
}

void goalFinishedCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("Waypoint finished (State %s)", state.toString().c_str());

  // The next waypoint should be sent
  doSendNextGoal = true;
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
  ROS_INFO("Feedback received.");

  // Just remember the current pose for track recording or track running
  currentPose = (*feedback).base_position;

  // If a track is currently run, remember the robots pose
  if(doRunTrack)
  {
    recordedRoutePoses.push_back(currentPose);
  }
}

void goalActiveCallback(){
  ROS_INFO("Goal went active.");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_driver");
	ros::NodeHandle n;

  // Initialize global storage path
  std::stringstream path;
  path << ros::package::getPath("waypoint") << "/storage/";
  storagePath = path.str();
  mkdir(storagePath.c_str(),0777);
  
  // Initialize all services
	ros::ServiceServer srvGetTracks     	= n.advertiseService("GetTracks", getTracks);
	ros::ServiceServer srvGetRoutes     	= n.advertiseService("GetRoutes", getRoutes);
	ros::ServiceServer srvStartNewTrack	  = n.advertiseService("StartNewTrack", startNewTrack);
	ros::ServiceServer srvSaveNewTrack	  = n.advertiseService("SaveNewTrack", saveNewTrack);
	ros::ServiceServer srvMarkWaypoint  	= n.advertiseService("MarkWaypoint", markWaypoint);
	ros::ServiceServer srvRunTrack      	= n.advertiseService("RunTrack", runTrack);
	ros::ServiceServer srvPlayBack  		  = n.advertiseService("PlayBack", playBack);
	ros::ServiceServer srvDeleteTrack	    = n.advertiseService("DeleteTrack", deleteTrack);
	ros::ServiceServer srvDeleteRoute	    = n.advertiseService("DeleteRoute", deleteRoute);
	ROS_INFO("Services initialized");

  // Initialize navigation server object
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", false);
  server = &ac;

  // Handle all service requests (blocks!)
	//ros::spin();

	ros::Rate loop(10);
	while (ros::ok())
	{

    // Handle all waiting callbacks
		ros::spinOnce();

    // Handle startNewTrack service command
    if(doRecordNewTrack)
    {

      // TODO currentPose wird noch nicht gefüllt!!!

      // Handle markWaypoint service command
      if(doMarkWaypoint)
      {
        ROS_INFO("Marking waypoint");
        recordedTrackPoses.push_back(currentPose);
        doMarkWaypoint = false;
      }

    }

    // Handle RunTrack service command
    if(doRunTrack)
    {

      // Handle RunTrack current waypoint reached
      if(doSendNextGoal)
      {

        // Have we reached the last waypoint?
        if(recordedTrackPoses.empty()){
          doRunTrack = false;
          // Save recorded route to bag file
          rosbag::Bag bag;
          std::stringstream filename;
          filename << storagePath << getCurrentTimeString() << "_R_" << recordedTrack.name << ".bag";
          bag.open(filename.str().c_str(), rosbag::bagmode::Write);
          for(unsigned int i = 0; i < recordedRoutePoses.size(); ++i)
          {
            bag.write("route", recordedRoutePoses[i].header.stamp, recordedRoutePoses[i]);
          }
          bag.close(); 
          recordedRoutePoses.clear();
          ROS_INFO("Routes saved.");
        }
        else
        {
          // Sending next goal to move_base
          move_base_msgs::MoveBaseGoal goal;
          goal.target_pose = recordedTrackPoses.front();
          goal.target_pose.header.stamp = ros::Time::now();
          server->sendGoal(goal, &goalFinishedCallback, &goalActiveCallback, &goalFeedbackCallback);
          recordedTrackPoses.pop_front();
          ROS_INFO("Waypoint sent as goal.");
        }

        doSendNextGoal = false;
      }

    }

    // Handle all waiting callbacks
    ros::spinOnce();

    // Sleep until loop rate is over
    loop.sleep();

	}

	return 0;
}
