#include "ros/ros.h"
#include "ros/time.h"
#include <sstream>

//#include <tf/transform_broadcaster.h>
//#include "std_msgs/String.h"
//#include "std_msgs/Int8.h"
//#include "nav_msgs/OccupancyGrid.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathfinder");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<nav_msgs::Path>("pathfinder/path", 1000);
  /*
  ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("talker/map",10);
  nav_msgs::OccupancyGrid map;
  map.header.frame_id="/talker/map";
  map.info.resolution=100;
  map.info.width=10;
  map.info.height = 10;
  geometry_msgs::Pose mapPose;
  mapPose.position.x=0;
  mapPose.position.y=0;
  mapPose.position.z=0;
  mapPose.orientation.x=1;
  mapPose.orientation.y=1;
  mapPose.orientation.z=0;
  mapPose.orientation.w=0;
  std::vector<int8_t> dataVec;
  for(int i = 0; i < 100; ++i){
    int8_t tmp;
    tmp = 1;
    if(i > 10 && i < 20)
      tmp = 0;
    dataVec.push_back(tmp);
  }
  map.data = dataVec; 
  */

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    //std_msgs::String msg;
    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str());

    nav_msgs::Path p;
    std::vector<geometry_msgs::PoseStamped> v;

    p.header.stamp = ros::Time::now();
    p.header.frame_id="/pathfinder/path";

    geometry_msgs::PoseStamped p1;
    geometry_msgs::PoseStamped p2;
    
    p1.header.frame_id = "/pathfinder/path";
    p1.header.stamp = ros::Time::now();
    p1.pose.position.x = 0;
    p1.pose.position.y = 0;
    p1.pose.position.z = 0;
    p1.pose.orientation.x = 0;
    p1.pose.orientation.y = 0;
    p1.pose.orientation.z = 0;
    p1.pose.orientation.w = 0;

    p2.header.frame_id = "/pathfinder/path";
    p2.header.stamp = ros::Time::now();
    p2.pose.position.x = 5;
    p2.pose.position.y = 5;
    p2.pose.position.z = 5;
    p2.pose.orientation.x = 0;
    p2.pose.orientation.y = 0;
    p2.pose.orientation.z = 0;
    p2.pose.orientation.w = 0;

    v.push_back(p1);
    v.push_back(p2);
    p.poses=v;

    chatter_pub.publish(p);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
