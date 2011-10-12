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
#include <list>
#include <numeric>
#include <iostream>
#include <sstream>
#include <wifi_comm/WiFiNeighboursList.h>

// defines
#define NODE_VERSION 0.01
#define BUFFSIZE 1024
#define LINK_HEADER "Table: Links"


std::string char2string(char *in)
{
    std::string str(in);
    return str;
}


/*
 * EXAMPLE OF THE INPUT
Table: Neighbors
IP address	SYM	MPR	MPRS	Will.	2 Hop Neighbors
192.168.10.4	NO	NO	NO	6	1
192.168.10.8	YES	NO	NO	3	1
192.168.10.7	YES	YES	NO	3	2
* 
Table: Links
Local IP	Remote IP	Hyst.	LQ	NLQ	Cost
192.168.10.6	192.168.10.8	0.00	1.000	1.000	1.000	
192.168.10.6	192.168.10.4	0.00	1.000	0.000	INFINITE	
192.168.10.6	192.168.10.7	0.00	1.000	1.000	1.000	
 * 
 */

void getNeighboursInfo(wifi_comm::WiFiNeighboursList * neighbours)
{
	FILE *ptr;
	char buf[BUFFSIZE];
	char link_cmd[] = "wget -q -O - localhost:8080/link";
	std::vector<std::string> neighboursList;
	std::string myIP;
	std::vector<int> signalList;

	// check if can be used
	if((ptr = popen(link_cmd, "r")) != NULL)
	{
		//read the first line
	    fgets(buf, BUFFSIZE, ptr);
	    
	    if(strcmp(buf, LINK_HEADER) == 0)
	    {
			ROS_INFO("buffer:%s",buf);
			ROS_WARN("Received Header is not a valid Header (%s)", LINK_HEADER);
			exit(1);
		}
		
		// read next line and ignore - column description
		fgets(buf, BUFFSIZE, ptr);
		
		// parsing variables
		char my_ip[16], other_ip[16];
		float hyst, lq, nlq, cost;
		std::string ip_str;
		
		while(fgets(buf, BUFFSIZE, ptr) != NULL && strlen(buf) > 1)
		{
			//ROS_INFO("rcv:%s", buf);
			sscanf (buf,"%s %s %f %f %f %f", my_ip, other_ip, &hyst, &lq, &nlq, &cost);
			//ROS_INFO("Parsed - my:%s   other:%s  H:%2.3f  LQ:%2.3f  NLQ:%2.3f  cost:%2.3f", my_ip, other_ip, hyst, lq, nlq, cost);
			
			// my IP		      			
			myIP = char2string(my_ip);
			// other IP
			ip_str = char2string(other_ip);
			
			// add to neigbour vector if link quality (LQ) is not zero
			// if LQ is zero the IP can be listed but is not connected
			if(lq > 0.0)
			{
				neighboursList.push_back(ip_str);
			
				int ilq = (int)(lq*100);
				// add to link quality vector
				signalList.push_back(ilq);
			}
		}
		
		neighbours->self_ip = myIP;
		// clear neigbours buffer from MSG
		neighbours->neighbours.clear();

		// create a vector<string>::iterator and set it to the beginning of the vector
		std::vector<std::string>::iterator it;
			
		int index;
		// Now, we iterate through the array until the iterator exceeds
		for(it = neighboursList.begin(); it != neighboursList.end(); it++)
		{
			//ROS_INFO("index:%d ", index);
			// list index
			index = int(it - neighboursList.begin());
			
			// one node
			wifi_comm::WiFiNeighbour * nod = new wifi_comm::WiFiNeighbour();
			
			nod->ip = (*it);
			nod->quality = signalList[index];
			neighbours->neighbours.push_back(*nod);
		}
	}
	pclose(ptr);
}


// *****************************************************************************
// Main function for the multiRobotCom node
int main( int argc, char** argv )
{
	ros::init(argc, argv, "wifi_discovery");
	ros::NodeHandle n;
	
	wifi_comm::WiFiNeighboursList neighbours;

	ros::Publisher list = n.advertise<wifi_comm::WiFiNeighboursList>("wifi_neighbours_list", 10);
	
	ROS_INFO("Wifi Discovery Node for ROS %.2f", NODE_VERSION);

	int freq;
	n.param("wifi_discovery_node/frequency", freq, 1);
	ros::Rate r(freq);
	// main loop
	while(ros::ok())
	{
		// Do stuff...
		getNeighboursInfo(&neighbours);

		// Publish data
		list.publish(neighbours);
		
		//ros::spinOnce();
		r.sleep();
	}

	return 0;
}

// EOF

