#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<nav_msgs::Path>("path", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    //std_msgs::String msg;

    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    nav_msgs::Path p;
    geometry_msgs::PoseStamped p1;
    geometry_msgs::PoseStamped p2;
    std::vector<geometry_msgs::PoseStamped> v;

    p1.header.frame_id = "1";
    p1.pose.position.x = 10;
    p1.pose.position.y = 10;
    p1.pose.position.z = 10;
    p1.pose.orientation.x = 0;
    p1.pose.orientation.y = 0;
    p1.pose.orientation.z = 0;
    p1.pose.orientation.w = 0;

    p2.header.frame_id = "2";
    p2.pose.position.x = 50;
    p2.pose.position.y = 0;
    p2.pose.position.z = 50;
    p2.pose.orientation.x = 0;
    p2.pose.orientation.y = 0;
    p2.pose.orientation.z = 0;
    p2.pose.orientation.w = 0;

    v.push_back(p1);
    v.push_back(p2);

    p.poses=v;

    p.header.frame_id="/talker/frame1";
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(p);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
