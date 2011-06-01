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


/*
 * gicp.h
 *
 *  Created on: Jan 23, 2011
 *      Author: engelhar
 */
  
#ifndef GICP_FALLBACK_H_
#define GICP_FALLBACK_H_

#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <Eigen/Core>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud_RGB;


void saveCloud(const char* filename, const PointCloud_RGB& pc, const unsigned int max_cnt = 10000);

void downSample(const PointCloud_RGB& src, PointCloud_RGB& to);

bool gicpfallback(const PointCloud_RGB& from, const PointCloud_RGB& to, Eigen::Matrix4f& transform);



#endif /* GICP_FALLBACK_H_ */
