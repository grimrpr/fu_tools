/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Gonçalo Cabrita and Pedro Sousa
*********************************************************************/
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <signal.h>
#include "wifi_comm/WiFiNeighboursList.h"
#include "wifi_comm/WiFiCommContainer.h"

// defines
#define NODE_VERSION 0.01

using namespace wifi_comm;

typedef struct
{
	int pid;
	std::string ip;

} ForeignRelay;

std::vector<ForeignRelay> ForeignRelays;


void openForeignRelay(char * ip)
{
	ForeignRelay newRelay;

	int pid = fork();
	if(pid==0)	// Child
	{
		char ros_master_uri[128];
		sprintf(ros_master_uri, "http://%s:11311", ip);

		execlp("rosrun", "rosrun", "foreign_relay", "foreign_relay", "adv", ros_master_uri, "/wifi_comm_rcv", "/wifi_comm_snd", (char*)0);
		
		ROS_WARN("Reached the end of foreign_relay to %s with pid %d", ip, getpid());
		exit(0);
	}
	else if(pid<0)	// Error
	{
		ROS_ERROR("Failed to open foreign_relay to %s", ip);
	}
	else		// Parent
	{
		newRelay.pid = pid;
		newRelay.ip = ip;
		ForeignRelays.push_back(newRelay);
		ROS_INFO("Successfully opened foreign_relay to %s with pid %d", ip, pid);
	}
	return;
}

void closeForeignRelay(char * ip)
{
	unsigned int i;

	for(i=0 ; i<ForeignRelays.size() ; i++)
	{
		if(ForeignRelays[i].ip.compare(ip)==0)
		{
			ROS_INFO("Killing foreign_relay to %s with pid %d", ip, ForeignRelays[i].pid);
			kill(ForeignRelays[i].pid, SIGTERM);
			ForeignRelays.erase(ForeignRelays.begin()+i);
			return;
		}
	}
	
	ROS_ERROR("Could not find foreign_relay to %s", ip);
	return;
}

void neighboursListUpdated(const wifi_comm::WiFiNeighboursListConstPtr& msg)
{
	unsigned int i, j;
	bool found;

  	// Now open and close foreign_relays!
	for(i=0 ; i<msg->neighbours.size() ; i++)
	{
		found = false;
		for(j=0 ; j<ForeignRelays.size() ; j++)
		{
			if(msg->neighbours[i].ip.compare(ForeignRelays[j].ip)==0)
			{
				found = true;
				break;
			}
		}
		if(!found) openForeignRelay((char*)msg->neighbours[i].ip.c_str());
	}

	for(i=0 ; i<ForeignRelays.size() ; i++)
	{
		found = false;
		for(j=0 ; j<msg->neighbours.size() ; j++)
		{
			if(ForeignRelays[i].ip.compare(msg->neighbours[j].ip)==0)
			{
				found = true;
				break;
			}
		}
		if(!found) closeForeignRelay((char*)ForeignRelays[i].ip.c_str());
	}
}


// *****************************************************************************
// Main function for the multiRobotCom node
int main( int argc, char** argv )
{
	ros::init(argc, argv, "wifi_comm");
	ros::NodeHandle n;
	
	ROS_INFO("Wifi Comm Node for ROS %.2f", NODE_VERSION);

	ros::Publisher pub = n.advertise<wifi_comm::WiFiCommContainer>("/wifi_comm_snd", 10);
	ros::Subscriber list_sub = n.subscribe("wifi_neighbours_list", 10, neighboursListUpdated);
 	ros::spin();

	return 0;
}

// EOF

