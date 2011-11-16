#include "ros/ros.h"
#include <ros/package.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include "waypoint/StartRoute.h"
#include "waypoint/SaveRoute.h"
#include "waypoint/GetRoutes.h"

#include "waypoint/StartTrack.h"
#include "waypoint/SaveTrack.h"
#include "waypoint/StartRouteOfTrack.h"
#include "waypoint/GetTracks.h"
#include "waypoint/MarkWaypoint.h"
#include "waypoint/DeleteTrack.h"

#include "waypoint/DeleteRoute.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <boost/foreach.hpp>

using namespace std;

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *server;
std::string storagePath;
waypoint::Track recordedTrack;
std::deque<geometry_msgs::PoseStamped> recordedTrackPoses;
std::deque<geometry_msgs::PoseWithCovarianceStamped> recordedRoutePoses;
geometry_msgs::PoseWithCovarianceStamped currentPoseWCVS;
std::vector<ros::Publisher> publishedRoutes;
bool doAdvertiseRouteTopics = true;
bool doRecordRoute = false;
bool doRecordTrack = false;
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

std::vector<std::string> getStorageFiles(char type, std::string name = std::string("-1WILLneverHAPPEN"))
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
      // Do we search all route files no matter if they belong to a track or not?
      if(name == "-1WILLneverHAPPEN" && type == 'R')
      {
        // Yes we do... 
        if(sscanf(str.c_str(),"%*[0-9]_%*[0-9]_%c", &t) == 1)
        {
          if(t == type)
            entries.push_back(str);
        }  
        continue;
      }
      // Do we search a standard file format (i.e. with track name suffix)?
      if(type == 'T' || !name.empty())
      {
        // Yes we do... is it a file in track format (i.e. with a track name suffix)?
        if(sscanf(str.c_str(),"%*[0-9]_%*[0-9]_%c_%[a-z-A-Z-0-9]", &t, n) == 2)
        {
	  if(type != 0 && type != t)
	  {
            continue;
          }
          if(!name.empty() && std::string(n) != name)
          {
            continue;
          }
          entries.push_back(str);
          continue;
        }
      }
      // Do we search a simple route file not belonging to any track?
      if(name.empty() && type == 'R')
      {
        // Yes we do... is it in simple route file format (i.e. without a track name suffix)?
        char dot;
        if(sscanf(str.c_str(),"%*[0-9]_%*[0-9]_%c%cbag", &t, &dot) == 2)
        {
          if(dot != '.') {continue;}
          entries.push_back(str);
          continue;
        }
      }
    }
  }

  closedir(directory);

  return entries;
}

bool getTracks(waypoint::GetTracks::Request &req, waypoint::GetTracks::Response &res)
{
  ROS_INFO("Service GetTracks called");

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
  ROS_INFO("Service GetRoutes called");
	
  std::vector<waypoint::Route> routes;

  std::vector<std::string> entries = getStorageFiles('R', req.track.name);

  for(unsigned int i = 0; i < entries.size(); ++i)
  {
    char timeDate[8];
    char timeClock[6];
    char name[1024];
    int s = sscanf(entries[i].c_str(),"%[0-9]_%[0-9]_%*c_%[a-z-A-Z-0-9]", timeDate, timeClock, name);
    waypoint::Route route;
    if(s == 3) {route.name = std::string(name);}
    if(s == 2 || s == 3)
    {
      route.time = std::string(timeDate) + std::string("_") + std::string(timeClock);    
      routes.push_back(route);
    }
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

bool startTrack(waypoint::StartTrack::Request &req, waypoint::StartTrack::Response &res)
{
  ROS_INFO("Service StartTrack called");

  if(doRecordTrack)
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
  doRecordTrack = true;
  doMarkWaypoint = true;

  res.track = recordedTrack;
  res.successful = true;
	return true;
}

bool saveTrack(waypoint::SaveTrack::Request &req, waypoint::SaveTrack::Response &res)
{
  ROS_INFO("Service SaveTrack called");

  if(recordedTrack.name != req.track.name || recordedTrack.time != req.track.time)
  {
    ROS_INFO("Given track is not the one that is currently recording");
    return true;
  }

  // Stop recording
  doRecordTrack = false;
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
    bag.write("track", ros::Time::now(), recordedTrackPoses[i]);
  }
  bag.close(); 

  // Clean recorded poses
  recordedTrackPoses.clear();
  
  res.successful = true;
	return true;
}

bool markWaypoint(waypoint::MarkWaypoint::Request &req, waypoint::MarkWaypoint::Response &res)
{
  ROS_INFO("Service MarkWaypoint called");

  if(doRecordTrack)
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

bool startRouteOfTrack(waypoint::StartRouteOfTrack::Request &req, waypoint::StartRouteOfTrack::Response &res)
{ 
  ROS_INFO("Service StartRouteOfTrack called");

  if(doRunTrack)
  {
    return true;
  }

  // Remember track, its name will be used for saving the route
  recordedTrack = req.track;

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

  // Start sending waypoints
  doRunTrack = true;
  doSendNextGoal = true;

  res.successful = true;
  return true;
}

bool deleteTrack(waypoint::DeleteTrack::Request &req, waypoint::DeleteTrack::Response &res)
{
  ROS_INFO("Service DeleteTrack called");

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

    // Make sure to stop advertising and publishing the deleted routes
    doAdvertiseRouteTopics = true;

    res.successful = true;
    return true;
  }

  ROS_INFO("This track coult not found");
  res.successful = false;
  return true;

}

bool deleteRoute(waypoint::DeleteRoute::Request &req, waypoint::DeleteRoute::Response &res)
{
  ROS_INFO("Service DeleteRoute called");

  std::vector<std::string> entries = getStorageFiles('R', req.route.name);
  for(unsigned int i = 0; i < entries.size(); ++i)
  {
    char timeDate[8];
    char timeClock[6];
    sscanf(entries[i].c_str(),"%[0-9]_%[0-9]_", timeDate, timeClock);
    if((std::string(timeDate) + std::string("_") + std::string(timeClock)) == req.route.time)
    {
      std::stringstream fileName;
      fileName << storagePath << entries[i];
      remove(fileName.str().c_str());
    }
  }

  // Make sure to stop advertising and publishing the deleted route
  doAdvertiseRouteTopics = true;

  res.successful = true;
  return true;
}

bool startRoute(waypoint::StartRoute::Request &req, waypoint::StartRoute::Response &res)
{
  ROS_INFO("Service StartRoute called");

  recordedRoutePoses.clear();
  doRecordRoute = true;
  
  res.successful = true;
  return true;
}

bool saveRoute(waypoint::SaveRoute::Request &req, waypoint::SaveRoute::Response &res)
{
  if(doRecordRoute)
  {
    doRecordRoute = false;
    res.route.time = getCurrentTimeString();
    
    if(recordedRoutePoses.empty())
    {
      ROS_INFO("No poses where recorded, saving anyway");
    }

    // Persist route file
    rosbag::Bag bag;
    std::stringstream filename;
    filename << storagePath << res.route.time << "_R.bag";
    bag.open(filename.str().c_str(), rosbag::bagmode::Write);
    for(unsigned int i = 0; i < recordedRoutePoses.size(); ++i)
    {
      bag.write("route", recordedRoutePoses[i].header.stamp, recordedRoutePoses[i]);
    }
    bag.close(); 

    // Clean recorded poses
    recordedRoutePoses.clear();

    // Make sure to publish the captured route
    doAdvertiseRouteTopics = true;
  }

  return true;
}

void goalFinishedCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("Callback called: Waypoint finished (State %s)", state.toString().c_str());

  // The next waypoint should be sent
  if(doRunTrack)
  {
    doSendNextGoal = true;
  }
}

void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
  ROS_INFO("Callback called: Feedback received.");
  // We have no interest in the feedback callback in this version
}

void goalActiveCallback(){
  ROS_INFO("Callback called: Goal went active.");
  // We have no interest in the active callback in this version
}

void currentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  currentPoseWCVS = (*msg);

  // Handle runs
  if(doRecordRoute || doRunTrack)
  {
    recordedRoutePoses.push_back(currentPoseWCVS);
  }
  
  // Handle DoMarkWaypoint
  if(doRecordTrack && doMarkWaypoint)
  {
    ROS_INFO("Marking waypoint");
    doMarkWaypoint = false;
    geometry_msgs::PoseStamped pose;
    pose.header = currentPoseWCVS.header;
    pose.pose = currentPoseWCVS.pose.pose;
    recordedTrackPoses.push_back(pose);
  }

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

  // Initialize pose listening
  ros::Subscriber sub = n.subscribe("amcl_pose", 1, currentPoseCallback);

  // Initialize all offered services
  ros::ServiceServer srvGetTracks 	    	= n.advertiseService("GetTracks", getTracks);
  ros::ServiceServer srvGetRoutes     		= n.advertiseService("GetRoutes", getRoutes);
  ros::ServiceServer srvStartTrack		= n.advertiseService("StartTrack", startTrack);
  ros::ServiceServer srvSaveTrack		= n.advertiseService("SaveTrack", saveTrack);
  ros::ServiceServer srvMarkWaypoint  		= n.advertiseService("MarkWaypoint", markWaypoint);
  ros::ServiceServer srvStartRouteOfTrack	= n.advertiseService("StartRouteOfTrack", startRouteOfTrack);
  ros::ServiceServer srvDeleteTrack		= n.advertiseService("DeleteTrack", deleteTrack);
  ros::ServiceServer srvDeleteRoute		= n.advertiseService("DeleteRoute", deleteRoute);
  ros::ServiceServer srvStartRoute		= n.advertiseService("StartRoute", startRoute);
  ros::ServiceServer srvSaveRoute		= n.advertiseService("SaveRoute", saveRoute);
  ROS_INFO("Services initialized");
 
  // Initialize navigation server object
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", false);
  server = &ac;
  
  // Handle all service requests
  ros::Rate loop(1);
  while (ros::ok())
  {

    // Handle all waiting callbacks
    ros::spinOnce();

    // Shall we (re-)advertise all route topics?
    // Do so, when system starts or routes where deleted or added
    if(doAdvertiseRouteTopics)
    {
      doAdvertiseRouteTopics = false;

      // Since we do not know which routes may have changed,
      // we cancel all advertised routes if there are any
      if(publishedRoutes.size() > 0)
      {
        for(unsigned int i = 0; i < publishedRoutes.size(); ++i)
        {
          publishedRoutes[i].shutdown();
        }
        publishedRoutes.clear();
        ROS_INFO("Canceled all advertised routes, readvertising...");
      }

      // Get all routes driven so far
      std::vector<std::string> availableFiles = getStorageFiles('R');
      for(unsigned int i = 0; i < availableFiles.size(); ++i)
      {
        // Advertise this route as a topic
        // Store publisher object for later useage
        std::string name("/wpd/");
        name.append(availableFiles[i].substr(0,availableFiles[i].size()-4));
	ROS_INFO("Advertising route %s", name.c_str());
        publishedRoutes.push_back(n.advertise<geometry_msgs::PoseArray>(name.c_str(), 1));
      }
    }
    else
    {
      // Check if someone listens to any route topics we previously advertised
      for(unsigned int i = 0; i < publishedRoutes.size(); ++i)
      {
        if(publishedRoutes[i].getNumSubscribers() > 0)
        {
          // Since someone is actually listening to that topic,
          // we have to publish the poses
          geometry_msgs::PoseArray poses;
          poses.header.frame_id = "/map";
          std::stringstream fileName;
          std::string name(publishedRoutes[i].getTopic());
          fileName << storagePath << name.substr(4,name.size()-4) << ".bag";
          if(fileExists(fileName.str()))
          {
            rosbag::Bag bag(fileName.str().c_str());
            rosbag::View view(bag, rosbag::TopicQuery("route"));
            BOOST_FOREACH(rosbag::MessageInstance const m, view)
            {
              geometry_msgs::PoseWithCovarianceStampedConstPtr pose = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
              if (pose != NULL)
                poses.poses.push_back(pose->pose.pose);
            }
            bag.close();
            ROS_INFO("Publishing route %s", publishedRoutes[i].getTopic().c_str());
            publishedRoutes[i].publish(poses);
          }
        }
      }
    }

    // Handle StartRouteOfTrack service command
    if(doRunTrack)
    {

      // Handle event current waypoint reached
      if(doSendNextGoal)
      {
 
        // Have we reached the last waypoint from that track?
        if(recordedTrackPoses.empty())
        {
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
          ROS_INFO("Route saved.");

	  recordedRoutePoses.clear();
          
          // Make sure, this new route will be advertised
          doAdvertiseRouteTopics = true;
        }
        else
        {
 	  // Still waypoints to go, sending next goal to move_base
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

    // Sleep until loop rate is over
    loop.sleep();
  }
}
