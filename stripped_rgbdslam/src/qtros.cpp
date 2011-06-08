/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "qtros.h"
#include "globaldefinitions.h"
QtROS::QtROS(int argc, char *argv[]) {
  std::cout << "Initializing Node...\n";
  ros::init(argc, argv, global_rosnode_name);
  n = new ros::NodeHandle();
  ROS_INFO("Connected to roscore");
}

void QtROS::run(){ 
  ros::Rate r(30); // 30 hz. Kinect has 30hz and we are far from processing every frame anyhow.
  while(ros::ok()) {
    ros::spinOnce(); 
    r.sleep();
  }
}
