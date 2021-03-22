/**
 * @file smooth_pose_traj.h
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

#ifndef INCLUDE_SMOOTH_POSE_TRAJ_H
#define INCLUDE_SMOOTH_POSE_TRAJ_H

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/math/interpolators/cubic_b_spline.hpp>

namespace smooth_pose_traj
{
class SmoothPoseTraj
{
public:
  /**
   * @brief Smooth a trajectory using boost cubic b spline
   * @details By default it aligns the xaxis with the direction of motion.
   * Set keep_xsign to true if you want the xaxis pointing in the same relative direction as the input
   * @param input_poses The input trajectory to smooth
   * @param point_spacing The point spacing of the smooth trajectory
   * @param keep_xsign If false the xais is align with the direction of motion. If true it will keep the xaxis point in the same relative direction as the input trajectory.
   */
  SmoothPoseTraj(const geometry_msgs::PoseArray& input_poses,
                 double point_spacing,
                 bool keep_xsign = false);
  virtual ~SmoothPoseTraj() = default;

  /**
   * @brief Smooth the trajectory and fill out the output_poses
   * @param output_poses The output trajectory
   * @param point_spacing The point spacing to use for output trajectory. If less than one it use the point spacing provided in the constructor.
   * @return True if successful, otherwise false
   */
  bool process(geometry_msgs::PoseArray& output_poses, double point_spacing = -1);

private:
  geometry_msgs::Pose interpPose(const geometry_msgs::Pose& P1, const geometry_msgs::Pose& P2, double alpha);
  geometry_msgs::Pose getPoseAtCrowDistance(double& t, double point_spacing, double& actual_distance);
  geometry_msgs::Pose getNPtAveragePose(const geometry_msgs::PoseArray& input_poses, int pose_index, int n_pts);
  bool align_x_to_next(geometry_msgs::PoseArray& poses);
  void qnormalize(geometry_msgs::Pose& P);

  double point_spacing_, total_distance_, max_t_;
  bool keep_xsign_{false};
  double align_sign_{1};

  boost::math::cubic_b_spline<double> sx_;
  boost::math::cubic_b_spline<double> sy_;
  boost::math::cubic_b_spline<double> sz_;
  boost::math::cubic_b_spline<double> sqx_;
  boost::math::cubic_b_spline<double> sqy_;
  boost::math::cubic_b_spline<double> sqz_;
  boost::math::cubic_b_spline<double> sqw_;

};  // end class SmoothPoseTraj

}  // namespace smooth_pose_traj
#endif  // INCLUDE_SMOOTH_POSE_TRAJ_H
