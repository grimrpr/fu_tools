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
#ifndef _WIFI_COMM_H
#define _WIFI_COMM_H

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <signal.h>
#include "wifi_comm/WiFiNeighboursList.h"
#include <boost/function.hpp>

#define STR_SIZE 128

namespace wifi_comm
{

	//! Foreign relay data structure, holds the foreign relay process pid and the host ip adress. 
	typedef struct
	{
		int pid;
		std::string ip;

	} ForeignRelay;

	/*! \class WiFiComm wifi_comm_lib.h "inc/wifi_comm_lib.h"
	 *  \brief C++ class for the WiFiComm multi-robot communication lib.
	 *
	 * This class allows for multi-robot communication through the use of OLSRD and the WifiComm discovery node.
	 */
	class WiFiComm
	{
		public:
		//! Deprecated constructor
		/*!
		 *
		 *  \param fp    C style callback function.
		 * 
		 */
		WiFiComm(void(*fp)(char * ip));
		//! Constructor
		/*!
		 *
		 *  \param f    Boost callback function.
		 * 
		 */
		WiFiComm(boost::function<void(char * ip)> f);
		//! Destructor
		~WiFiComm();
	
		//! Concat topic and ip
		/*!
		*  This function concatenates a topic and an ip adress thus creating a unique topic to be advertised.
		*
		*  \param final_topic   C-string containing the final topic.
		*  \param topic  		C-string containing the base topic name.
		*  \param ip  			C-string containing the ip adress.
		* 
		*  \return A pointer to the final topic.
		*/
		static char * concatTopicAndIp(char * final_topic, char * topic, const char * ip);

		//! Open foreign relay
		/*!
		*  This function opens a foreign relay to publish a local topic on a foreign roscore at the given ip adress.
		*
		*  \param ip  			 C-string containing the ip adress.
		*  \param topic  		 C-string containing the topic name.
		*  \param public_publish Wether to publish an ip specific topic or not.
		*  \param append_my_ip   Wether to append my ip to the topic on the receiving end or not
		* 
		*  \return Foreign roscore ip adress.
		*/
		std::string * openForeignRelay(char * ip, char * topic, bool public_publish=true, bool append_my_ip=true);
		
		//! Open foreign relay
		/*!
		*  This function opens a foreign relay to publish a local topic on a foreign roscore at the given ip adress.
		*
		*  \param ip  			 C-string containing the ip adress.
		*  \param topic  		 C-string containing the local topic name.
		*  \param topic  		 C-string containing the foreign topic name.
		* 
		*  \return Foreign roscore ip adress.
		*/
		std::string * openForeignRelay(char * ip, char * local_topic, char * foreign_topic);

		//! List of neighbour robots
		wifi_comm::WiFiNeighboursList neighbours_list_;
	
		private:
		//! Node handle
		ros::NodeHandle n_;
		//! Subscriber for the WiFiNeighboursList message
		ros::Subscriber neighbours_sub_;

		//! Vector of foreign relays
		std::vector<ForeignRelay> foreign_relays_;

		//! Close foreign relay
		/*!
		*  This function closes an existing foreign relay.
		*
		*  \param ip  	The ip adress of the foreign relays to close.
		*
		*/
		void closeForeignRelay(char * ip);

		//! Goal callback
		/*!
		 *  This function is a callback for receiving WiFiNeighboursList messages.
		 *
		 *  \param msg    Received WiFiNeighboursList message.
		 * 
		 */
		void neighboursListUpdated(const wifi_comm::WiFiNeighboursListConstPtr& msg);
	
		//! Whether we are using the boost callback function or not
		bool using_boost_;
		//! C style callback function
		void(*c_callback_)(char * ip);
		//! Boost callback function
		boost::function<void(char*)> boost_callback_;
	};

} // namespace

#endif        /* _WIFI_COMM_H */

// EOF

