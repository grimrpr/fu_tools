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

nav_msgs::Path path;
std::vector<geometry_msgs::PoseStamped> vecPoses;
ros::Publisher path_pub;
ros::Subscriber pose_sub;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

  ROS_ERROR("poseCallback: publishing path...");

  ROS_ERROR("%f",msg->pose.position.x);


  path.header.stamp = ros::Time::now();

  path.poses.push_back(*msg);

  path_pub.publish(path);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathfinder");

  ros::NodeHandle n;

  pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/stripped_rgbdslam_1/currentPose",100,poseCallback);
  path_pub = n.advertise<nav_msgs::Path>("/pathfinder/path", 1000);

  // Initialize path object
  path.header.frame_id="/pathfinder/path";
  path.poses=vecPoses;

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

  while (ros::ok())
  {
    //std_msgs::String msg;
    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str());

    /*
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
    */

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
