/**
 * @file smooth_pose_traj.cpp
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <smooth_pose_traj/interleave_pose_traj.hpp>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>

namespace smooth_pose_traj
{
  void InterleavePoseTraj(std::vector<geometry_msgs::PoseArray>& tool_paths, double raster_spacing)
  {

    printf("tool_paths.size() = %ld", tool_paths.size());
    for(size_t i=0; i<tool_paths.size()-1; i++)// duplicate all but the last raster
      {
	geometry_msgs::PoseArray seg = tool_paths[i];
	geometry_msgs::PoseArray new_seg;
	new_seg.header = seg.header;
	double half_raster = raster_spacing/2.0;
	// here we assume the waypoints for all rasters point in same general direction (left to right or right to left)
	// however the ordering goes left to right for the first stroke then right to left for the secong stroke
	// to have the interleave do the same, we reverse the ordering for odd segments
	if(tool_paths.size()%2) 
	  {
	    printf("even\n");
	    for( size_t j=0; j<seg.poses.size(); j++)
	      {
		printf("j = %d\n",j);
		geometry_msgs::Pose IP = seg.poses[j];
		// offset xyz position along y-axis by half the raster_spacing
		Eigen::Quaterniond Q(IP.orientation.w, IP.orientation.x, IP.orientation.y, IP.orientation.z);
		Eigen::Matrix3d R = Q.toRotationMatrix();
		Eigen::Vector3d y_axis(R.col(1));
		IP.position.x += half_raster * y_axis.x();
		IP.position.y += half_raster * y_axis.y();
		IP.position.z += half_raster * y_axis.z();
		new_seg.poses.push_back(IP);
	      }
	  }
	else
	  {
	    printf("odd\n");
	    for( int j=static_cast<int>(seg.poses.size()-1); j>=0; j--)
	      {
		printf("j = %d\n",j);
		geometry_msgs::Pose IP = seg.poses[j];
		// offset xyz position along y-axis by half the raster_spacing
		Eigen::Quaterniond Q(IP.orientation.w, IP.orientation.x, IP.orientation.y, IP.orientation.z);
		Eigen::Matrix3d R = Q.toRotationMatrix();
		Eigen::Vector3d y_axis(R.col(1));
		IP.position.x += half_raster * y_axis.x();
		IP.position.y += half_raster * y_axis.y();
		IP.position.z += half_raster * y_axis.z();
		new_seg.poses.push_back(IP);
	      }

	  }
	// append the new interleaved
	tool_paths.push_back(new_seg);
      }
  }// end of function interleave_pose_traj()
}  // namespace smooth_pose_traj
