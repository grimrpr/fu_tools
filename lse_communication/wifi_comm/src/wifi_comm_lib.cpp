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
#include "wifi_comm/wifi_comm_lib.h"

wifi_comm::WiFiComm::WiFiComm(void(*fp)(char * ip)) : n_()
{
	ROS_ERROR("Using the constructor wifi_comm::WiFiComm::WiFiComm(void(*)(char*)) is deprecated. Please change to wifi_comm::WiFiComm::WiFiComm(boost::function<void(char*)>)");

	neighbours_sub_ = n_.subscribe("wifi_neighbours_list", 10, &WiFiComm::neighboursListUpdated, this);

    c_callback_ = fp;
    using_boost_ = false;
}

wifi_comm::WiFiComm::WiFiComm(boost::function<void(char * ip)> f) : n_()
{
	neighbours_sub_ = n_.subscribe("wifi_neighbours_list", 10, &WiFiComm::neighboursListUpdated, this);

    boost_callback_ = f;
    using_boost_ = true;
}

wifi_comm::WiFiComm::~WiFiComm()
{
	ROS_INFO("Killing all active foreign relays before exiting...");
	for(int i=0 ; i<foreign_relays_.size() ; i++)
	{
		closeForeignRelay((char*)foreign_relays_[i].ip.c_str());
	}
}

char * wifi_comm::WiFiComm::concatTopicAndIp(char * final_topic, char * topic, const char * ip)
{
	sprintf(final_topic, "%s_%s", topic, ip);

	for(int i=0 ; i<strlen(final_topic) ; i++)
	{
		if(final_topic[i]=='.') final_topic[i]='_'; 
	}
	return final_topic;
}

std::string * wifi_comm::WiFiComm::openForeignRelay(char * ip, char * topic, bool public_publish, bool append_my_ip)
{
	ROS_ERROR("Using wifi_comm::WiFiComm::openForeignRelay(char * ip, char * topic, bool public_publish, bool append_my_ip) is deprecated. Please change to wifi_comm::WiFiComm::openForeignRelay(char * ip, char * local_topic, char * foreign_topic).");

	ForeignRelay newRelay;

	int pid = fork();
	if(pid==0)	// Child
	{
		char foreign_topic[STR_SIZE];
		char local_topic[STR_SIZE];
		
		if(public_publish) strcpy(local_topic, topic);
		else concatTopicAndIp(local_topic, topic, ip);

		char ros_master_uri[STR_SIZE];
		sprintf(ros_master_uri, "http://%s:11311", ip);
		execlp("rosrun", "rosrun", "foreign_relay", "foreign_relay", "adv", ros_master_uri, append_my_ip ? concatTopicAndIp(foreign_topic, topic, (char*)neighbours_list_.self_ip.c_str()) : topic, local_topic, (char*)0);
		
		ROS_WARN("Reached the end of foreign_relay to %s on the topic %s with pid %d", ip, topic, getpid());
		exit(0);
	}
	else if(pid<0)	// Error
	{
		ROS_ERROR("Failed to open foreign_relay to %s on the topic %s", ip, topic);
		return NULL;
	}
	else		// Parent
	{
		newRelay.pid = pid;
		newRelay.ip = ip;
		foreign_relays_.push_back(newRelay);
		ROS_INFO("Successfully opened foreign_relay to %s on the topic %s with pid %d", ip, topic, pid);
		return &newRelay.ip;
	}
	return NULL;
}

std::string * wifi_comm::WiFiComm::openForeignRelay(char * ip, char * local_topic, char * foreign_topic)
{
	ForeignRelay newRelay;

	int pid = fork();
	if(pid==0)	// Child
	{
		char ros_master_uri[STR_SIZE];
		sprintf(ros_master_uri, "http://%s:11311", ip);
		execlp("rosrun", "rosrun", "foreign_relay", "foreign_relay", "adv", ros_master_uri, foreign_topic, local_topic, (char*)0);
		
		ROS_WARN("Reached the end of foreign_relay to %s on the topic %s with pid %d", ip, foreign_topic, getpid());
		exit(0);
	}
	else if(pid<0)	// Error
	{
		ROS_ERROR("Failed to open foreign_relay to %s on the topic %s", ip, foreign_topic);
		return NULL;
	}
	else		// Parent
	{
		newRelay.pid = pid;
		newRelay.ip = ip;
		foreign_relays_.push_back(newRelay);
		ROS_INFO("Successfully opened foreign_relay to %s on the topic %s with pid %d", ip, foreign_topic, pid);
		return &newRelay.ip;
	}
	return NULL;
}

void wifi_comm::WiFiComm::closeForeignRelay(char * ip)
{
	unsigned int i;

	for(i=0 ; i<foreign_relays_.size() ; i++)
	{
		if(foreign_relays_[i].ip.compare(ip)==0)
		{
			// Apparently killing one is not enough! Do the child killing:
			char cmd[STR_SIZE];
			sprintf(cmd, "ps -o pid= --ppid %d", foreign_relays_[i].pid);			

			char buf[STR_SIZE];
			int child_pid;
			FILE * ptr;
			if((ptr = popen(cmd, "r")) != NULL)
			{
				fscanf(ptr, "%d", &child_pid);
				if(kill(child_pid, SIGTERM)!=0) ROS_WARN("Unable to kill foreign_relay child to %s with pid %d", ip, child_pid);
				if(kill(foreign_relays_[i].pid, SIGTERM)!=0) ROS_WARN("Unable to kill foreign_relay to %s with pid %d", ip, foreign_relays_[i].pid);
			}
			else
			{
				ROS_WARN("Unable to kill foreign_relay child to %s with ppid %d", ip, foreign_relays_[i].pid);
			}
			pclose(ptr);

			ROS_INFO("Closed foreign_relay to %s with pid %d", ip, foreign_relays_[i].pid);			

			foreign_relays_.erase(foreign_relays_.begin()+i);
		}
	}
	return;
}

void wifi_comm::WiFiComm::neighboursListUpdated(const wifi_comm::WiFiNeighboursListConstPtr& msg)
{
	unsigned int i, j, ind;
	bool found;

	// We store the msg so that we can access the list and size() without having to subscribe again
	neighbours_list_.self_ip.assign(msg->self_ip);
	neighbours_list_.neighbours.clear();
	for(i=0 ; i<msg->neighbours.size() ; i++) neighbours_list_.neighbours.push_back(msg->neighbours[i]);

  	// Now open and close foreign_relays!
	for(i=0 ; i<msg->neighbours.size() ; i++)
	{
		found = false;
		for(j=0 ; j<foreign_relays_.size() ; j++)
		{
			if(msg->neighbours[i].ip.compare(foreign_relays_[j].ip)==0)
			{
				found = true;
				break;
			}
		}
		// Trigger callback
		if(!found)
		if(using_boost_) boost_callback_((char*)msg->neighbours[i].ip.c_str());
		else (c_callback_)((char*)msg->neighbours[i].ip.c_str());
	}

	for(i=0 ; i<foreign_relays_.size() ; i++)
	{
		found = false;
		for(j=0 ; j<msg->neighbours.size() ; j++)
		{
			if(foreign_relays_[i].ip.compare(msg->neighbours[j].ip)==0)
			{
				found = true;
				break;
			}
		}
		if(!found) closeForeignRelay((char*)foreign_relays_[i].ip.c_str());
	}
}

// EOF

