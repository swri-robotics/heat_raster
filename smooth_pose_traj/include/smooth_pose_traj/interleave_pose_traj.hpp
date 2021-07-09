/**
 * @file interleave_pose_traj.h
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

#ifndef INCLUDE_INTERLEAVE_POSE_TRAJ_H
#define INCLUDE_INTERLEAVE_POSE_TRAJ_H

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <geometry_msgs/PoseArray.h>

namespace smooth_pose_traj
{
  /**
   * @brief Appends an interleaved set tool_paths to the provided tool_paths
   * @param tool_paths The input trajectory to interleave
   * @param raster_spacing
   */
  void InterleavePoseTraj(tool_path_planner::ToolPaths tool_paths, double raster_spacing);

}  // namespace smooth_pose_traj
#endif  // INCLUDE_INTERLEAVE_POSE_TRAJ_H
