#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

tf::TransformBroadcaster *odom_broadcaster;
int i = 0;

void odom_msg_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

	if(++i > 100)
	{
		ROS_DEBUG("Publishing odometry transformation...");
		i = 0; 
	}

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	x += msg->pose.pose.position.x;
	y += msg->pose.pose.position.y;
	th += tf::getYaw(msg->pose.pose.orientation);

	ROS_INFO("x: %f, y: %f, th: %f", x, y, th);

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = msg->header.stamp;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint"; //"base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	odom_broadcaster->sendTransform(odom_trans);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "odom_pub");

	ros::NodeHandle n;
	odom_broadcaster = new tf::TransformBroadcaster;
	ros::Subscriber sub = n.subscribe("/odom", 1, odom_msg_callback);

	ROS_INFO("Odometry transform publisher is now running...");

	ros::spin();
}
