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
#include <ros/ros.h>
#include <boost/format.hpp>
#include <mutex>
#include <thread>
#include <smooth_pose_traj/SmoothPoseTrajectory.h>  // the service
#include <smooth_pose_traj/smooth_pose_traj.hpp>    // the .hpp for this cpp file

static const std::string POSE_TRAJ_SMOOTHER = "pose_trajectory_smoother";
static ros::ServiceServer smooth_pose_traj_srv;

bool smoothPoseTrajCB(smooth_pose_traj::SmoothPoseTrajectory::Request& req,
                      smooth_pose_traj::SmoothPoseTrajectory::Response& res)
{
  smooth_pose_traj::SmoothPoseTraj SPT(req.input_poses, req.point_spacing);
  return (SPT.process(res.output_poses, req.point_spacing));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_trajectory_smoother");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh;
  spinner.start();
  smooth_pose_traj_srv = nh.advertiseService(POSE_TRAJ_SMOOTHER, &smoothPoseTrajCB);
  ros::waitForShutdown();
  return 0;
}
