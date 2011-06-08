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

/* registration main:
 * Create
 * - a Qt Application
 * - a ROS Node,
 * - a ROS listener, listening to and processing kinect data
 * - Let them communicate internally via QT Signals
 */
#include "openni_listener.h"
#include "qtros.h"
#include <QApplication>
#include <QObject>
#include "qtcv.h"
#include <Eigen/Core>
#include "globaldefinitions.h"

int main(int argc, char** argv)
{

  QtROS qtRos(argc, argv); //Thread object, to run the ros event processing loop in parallel to the qt loop

  GraphManager graph_mgr(qtRos.getNodeHandle()); 
  
  //add kinect device number to global topics if more than one kinect shall run
  char* device_number;
  if(argc>0){
    device_number = argv[1];
  }
  else{
    device_number = "";
  }
  std::string cam("/camera");
  cam += device_number;

  std::string topic_image_mono_str(cam + global_topic_image_mono);
  std::string topic_image_depth_str(cam + global_topic_image_depth);
  std::string topic_image_points_str(cam + global_topic_points);

  //Instantiate the kinect image listener
  OpenNIListener kinect_listener(qtRos.getNodeHandle(), &graph_mgr,
                                 topic_image_mono_str.c_str(),
                                 topic_image_depth_str.c_str(),
                                 topic_image_points_str.c_str(),
                                 global_feature_extractor_type, //FAST is really fast but the Keypoints are not robust
                                 global_feature_detector_type);

  // Run main loop.
  qtRos.start();

#ifdef USE_ICP_BIN
  ROS_INFO("ICP activated via external binary");
#endif
#ifdef USE_ICP_CODE
  ROS_INFO("ICP activated via linked library");
#endif

  ros::Rate r(0.5);
  while(ros::ok()){
    kinect_listener.getOneFrame();
    r.sleep();
  }

  ros::waitForShutdown();
}
