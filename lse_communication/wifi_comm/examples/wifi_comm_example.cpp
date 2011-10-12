/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*!
 *
 * \file wifi_comm_example.cpp
 *
 * \brief Example for the wifi_comm ROS pkg.
 *
 * \author Gonçalo Cabrita
 * \author Pedro Sousa
 * \date 14/2010
 * \version 0.1
 *
 * \bug none discovered
 *
 * \note
 *
 */

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <signal.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <wifi_comm/WiFiNeighboursList.h>
#include "std_msgs/String.h"

#include "wifi_comm/wifi_comm_lib.h"

using namespace wifi_comm;

WiFiComm * myComm;
ros::NodeHandle * n;

std::vector<ros::Subscriber> subs;

void helloWorldCallback(std::string ip, const std_msgs::StringConstPtr& msg)
{
	ROS_INFO("Received [%s] from %s", msg->data.c_str(), ip.c_str());
}

void robotJoinedNetwork(char * ip)
{
	// Send
	myComm->openForeignRelay(ip, "/wifi_hello_world", true);

	// Receive
	char topic[128];
	ros::Subscriber sub = n->subscribe<std_msgs::String>(WiFiComm::concatTopicAndIp(topic, "/wifi_hello_world", ip), 10, boost::bind(helloWorldCallback, std::string(ip), _1));
	subs.push_back(sub);
}

// *****************************************************************************
// Main function for the multiRobotComm example node
int main( int argc, char** argv )
{
	ros::init(argc, argv, "wifi_comm_example");
	n = new ros::NodeHandle();

	myComm = new WiFiComm(robotJoinedNetwork);

	ros::Publisher hello_world = n->advertise<std_msgs::String>("wifi_hello_world", 10);
		
	ros::Rate r(0.5);
	while(ros::ok())
	{
		std_msgs::String msg;
    		msg.data = "Hello World!";
    		hello_world.publish(msg);

		ros::spinOnce();
		r.sleep();	
	}

	delete myComm;

	return 0;
}

// EOF

