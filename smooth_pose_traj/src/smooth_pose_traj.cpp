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
#include <smooth_pose_traj/smooth_pose_traj.hpp>
#include <eigen3/Eigen/Eigen>

namespace smooth_pose_traj
{
void SmoothPoseTraj::qnormalize(geometry_msgs::Pose& P)
{
  Eigen::Quaterniond Q(P.orientation.w, P.orientation.x, P.orientation.y, P.orientation.z);
  Q.normalize();
  P.orientation.x = Q.x();
  P.orientation.y = Q.y();
  P.orientation.z = Q.z();
  P.orientation.w = Q.w();
}  // end qnormalize()

geometry_msgs::Pose SmoothPoseTraj::interpPose(const geometry_msgs::Pose& P1,
                                               const geometry_msgs::Pose& P2,
                                               double alpha)
{
  geometry_msgs::Pose P;
  if (alpha < 0.0)
    alpha = 0.0;
  if (alpha > 1.0)
    alpha = 1.0;
  P.position.x = (1 - alpha) * P1.position.x + alpha * P2.position.x;
  P.position.y = (1 - alpha) * P1.position.y + alpha * P2.position.y;
  P.position.z = (1 - alpha) * P1.position.z + alpha * P2.position.z;
  P.orientation.x = (1 - alpha) * P1.orientation.x + alpha * P2.orientation.x;
  P.orientation.y = (1 - alpha) * P1.orientation.y + alpha * P2.orientation.y;
  P.orientation.z = (1 - alpha) * P1.orientation.z + alpha * P2.orientation.z;
  P.orientation.w = (1 - alpha) * P1.orientation.w + alpha * P2.orientation.w;

  // Normalize Quaternion
  qnormalize(P);

  return (P);
}
geometry_msgs::Pose SmoothPoseTraj::getNPtAveragePose(const geometry_msgs::PoseArray& input_poses,
                                                      int pose_index,
                                                      int n_pts)
{
  int n = static_cast<int>(input_poses.poses.size());
  geometry_msgs::Pose p;
  p.position.x = 0;
  p.position.y = 0;
  p.position.z = 0;
  p.orientation.x = 0;
  p.orientation.y = 0;
  p.orientation.z = 0;
  p.orientation.w = 0;

  if (pose_index == 0 || pose_index == n - 1)
    return (input_poses.poses[static_cast<std::size_t>(pose_index)]);

  int total = 0;
  int n_each_side = n_pts / 2;
  int start = pose_index - n_each_side;
  int stop = pose_index + n_each_side;
  if (start < 0)
    start = 0;
  if (stop > n)
    stop = n;
  for (size_t i = static_cast<size_t>(start); i < static_cast<size_t>(stop); i++)
  {
    p.position.x += input_poses.poses[i].position.x;
    p.position.y += input_poses.poses[i].position.y;
    p.position.z += input_poses.poses[i].position.z;
    p.orientation.x += input_poses.poses[i].orientation.x;
    p.orientation.y += input_poses.poses[i].orientation.y;
    p.orientation.z += input_poses.poses[i].orientation.z;
    p.orientation.w += input_poses.poses[i].orientation.w;
    total++;
  }
  p.position.x = p.position.x / total;
  p.position.y = p.position.y / total;
  p.position.z = p.position.z / total;
  p.orientation.x = p.orientation.x / total;
  p.orientation.y = p.orientation.y / total;
  p.orientation.z = p.orientation.z / total;
  p.orientation.w = p.orientation.w / total;

  return (p);
}

SmoothPoseTraj::SmoothPoseTraj(const geometry_msgs::PoseArray& input_poses,
                               double point_spacing,
                               bool keep_xsign)
  : point_spacing_(point_spacing)
  , keep_xsign_(keep_xsign)
  , align_sign_(1)
{
  // fit spline to each component
  std::vector<double> x, y, z, qx, qy, qz, qw;
  int n = static_cast<int>(input_poses.poses.size());
  max_t_ = static_cast<double>(n - 2);

  if (keep_xsign_)
  {
    Eigen::Vector3d start_x_axis = Eigen::Quaterniond(input_poses.poses[0].orientation.w, input_poses.poses[0].orientation.x, input_poses.poses[0].orientation.y, input_poses.poses[0].orientation.z).toRotationMatrix().col(0);
    Eigen::Vector3d start_dir = Eigen::Vector3d(input_poses.poses[1].position.x - input_poses.poses[0].position.x,
                                                input_poses.poses[1].position.y - input_poses.poses[0].position.y,
                                                input_poses.poses[1].position.z - input_poses.poses[0].position.z).normalized();

    align_sign_ = (start_x_axis.dot(start_dir) < 0) ? -1 : 1;
  }

  // Its a mess to handle all the short path cases. Here I just inserted extra points so the spline will run.
  if (n == 2)  // insert 3 extra points equally spaced
  {
    for (int i = 0; i < 5; i++)
    {
      double alpha = i / 4;
      geometry_msgs::Pose p = interpPose(input_poses.poses[0], input_poses.poses[1], alpha);
      x.push_back(p.position.x);
      y.push_back(p.position.y);
      z.push_back(p.position.z);
      qx.push_back(p.orientation.x);
      qy.push_back(p.orientation.y);
      qz.push_back(p.orientation.z);
      qw.push_back(p.orientation.w);
    }
  }
  else if (n == 3)  // insert one point between each pair to make 5
  {
    for (int i = 0; i < 3; i++)
    {
      double alpha = i / 2;
      geometry_msgs::Pose p = interpPose(input_poses.poses[0], input_poses.poses[1], alpha);
      x.push_back(p.position.x);
      y.push_back(p.position.y);
      z.push_back(p.position.z);
      qx.push_back(p.orientation.x);
      qy.push_back(p.orientation.y);
      qz.push_back(p.orientation.z);
      qw.push_back(p.orientation.w);
    }
    for (int i = 1; i < 3; i++)
    {
      double alpha = i / 2;
      geometry_msgs::Pose p = interpPose(input_poses.poses[1], input_poses.poses[2], alpha);
      x.push_back(p.position.x);
      y.push_back(p.position.y);
      z.push_back(p.position.z);
      qx.push_back(p.orientation.x);
      qy.push_back(p.orientation.y);
      qz.push_back(p.orientation.z);
      qw.push_back(p.orientation.w);
    }
  }
  else if (n == 4)  // here we add a point between every pair and get 7 total points
  {
    for (int i = 0; i < 3; i++)
    {
      double alpha = i / 2;
      geometry_msgs::Pose p = interpPose(input_poses.poses[0], input_poses.poses[1], alpha);
      x.push_back(p.position.x);
      y.push_back(p.position.y);
      z.push_back(p.position.z);
      qx.push_back(p.orientation.x);
      qy.push_back(p.orientation.y);
      qz.push_back(p.orientation.z);
      qw.push_back(p.orientation.w);
    }
    for (int i = 1; i < 3; i++)
    {
      double alpha = i / 2;
      geometry_msgs::Pose p = interpPose(input_poses.poses[1], input_poses.poses[2], alpha);
      x.push_back(p.position.x);
      y.push_back(p.position.y);
      z.push_back(p.position.z);
      qx.push_back(p.orientation.x);
      qy.push_back(p.orientation.y);
      qz.push_back(p.orientation.z);
      qw.push_back(p.orientation.w);
    }
    for (int i = 1; i < 3; i++)
    {
      double alpha = i / 2;
      geometry_msgs::Pose p = interpPose(input_poses.poses[2], input_poses.poses[3], alpha);
      x.push_back(p.position.x);
      y.push_back(p.position.y);
      z.push_back(p.position.z);
      qx.push_back(p.orientation.x);
      qy.push_back(p.orientation.y);
      qz.push_back(p.orientation.z);
      qw.push_back(p.orientation.w);
    }
  }
  else  // 5 or more, and the spline works fine, just add all the points
  {
    for (int i = 0; i < n; i++)
    {
      geometry_msgs::Pose p = getNPtAveragePose(input_poses, i, 9);
      x.push_back(p.position.x);
      y.push_back(p.position.y);
      z.push_back(p.position.z);
      qx.push_back(p.orientation.x);
      qy.push_back(p.orientation.y);
      qz.push_back(p.orientation.z);
      qw.push_back(p.orientation.w);
    }
  }
  sx_ = boost::math::cubic_b_spline<double>(x.begin(), x.end(), 0.0, 1.0);
  sy_ = boost::math::cubic_b_spline<double>(y.begin(), y.end(), 0.0, 1.0);
  sz_ = boost::math::cubic_b_spline<double>(z.begin(), z.end(), 0.0, 1.0);
  sqx_ = boost::math::cubic_b_spline<double>(qx.begin(), qx.end(), 0.0, 1.0);
  sqy_ = boost::math::cubic_b_spline<double>(qy.begin(), qy.end(), 0.0, 1.0);
  sqz_ = boost::math::cubic_b_spline<double>(qz.begin(), qz.end(), 0.0, 1.0);
  sqw_ = boost::math::cubic_b_spline<double>(qw.begin(), qw.end(), 0.0, 1.0);

  Eigen::Vector3d pi(sx_(0.0), sy_(0), sz_(0));
  total_distance_ = 0.0;
  for (double t = 1.0; t < max_t_; t += 1.0)
  {
    Eigen::Vector3d p(sx_(t), sy_(t), sz_(t));
    total_distance_ += (p - pi).norm();
    pi = p;
  }
}  // end constructor for SmoothPoseTraj

bool SmoothPoseTraj::process(geometry_msgs::PoseArray& output_poses, double point_spacing)
{
  if (point_spacing < 0)
    point_spacing = point_spacing_;

  output_poses.poses.clear();

  // add start pose
  Eigen::Quaterniond Q(sqw_(0), sqx_(0), sqy_(0), sqz_(0));
  Q.normalize();
  geometry_msgs::Pose P1;
  P1.position.x = sx_(0);
  P1.position.y = sy_(0);
  P1.position.z = sz_(0);
  P1.orientation.x = Q.x();
  P1.orientation.y = Q.y();
  P1.orientation.z = Q.z();
  P1.orientation.w = Q.w();
  output_poses.poses.push_back(P1);

  double t = 0.0;
  while (t < max_t_)
  {
    double D;
    geometry_msgs::Pose P = getPoseAtCrowDistance(t, point_spacing, D);
    if (D > .5 * point_spacing)  // only add the new pose if its a significant fraction of the desired spacing
      output_poses.poses.push_back(P);
  }
  align_x_to_next(output_poses);
  return (true);
}  // end process()

bool SmoothPoseTraj::align_x_to_next(geometry_msgs::PoseArray& poses)
{
  Eigen::Vector3d x_axis;
  for (size_t i = 0; i < poses.poses.size(); i++)
  {
    // convert quat to a rotation matrix
    Eigen::Quaterniond Q(poses.poses[i].orientation.w,
                         poses.poses[i].orientation.x,
                         poses.poses[i].orientation.y,
                         poses.poses[i].orientation.z);
    Eigen::Matrix3d R = Q.toRotationMatrix();

    // keep normal as z axis
    Eigen::Vector3d z_axis = R.col(2); //R(0, 2), R(1, 2), R(2, 2));

    // compute x to point from point i to point i+1
    if (i < poses.poses.size() - 1)  // last point keeps previous x axis
    {
      x_axis = align_sign_ * Eigen::Vector3d(poses.poses[i + 1].position.x - poses.poses[i].position.x,
                                             poses.poses[i + 1].position.y - poses.poses[i].position.y,
                                             poses.poses[i + 1].position.z - poses.poses[i].position.z).normalized();
    }

    // compute y = z-cross-x
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
    x_axis = y_axis.cross(z_axis).normalized();
    z_axis.normalize();

    // form rotation matrix from the three vectors
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    // convert rotation matrix to quat
    Eigen::Quaterniond Q2(R);
    Q2.normalize();
    poses.poses[i].orientation.w = Q2.w();
    poses.poses[i].orientation.x = Q2.x();
    poses.poses[i].orientation.y = Q2.y();
    poses.poses[i].orientation.z = Q2.z();
  }
  return (true);
}

geometry_msgs::Pose SmoothPoseTraj::getPoseAtCrowDistance(double& t, double point_spacing, double& actual_distance)
{
  geometry_msgs::Pose P;
  Eigen::Vector3d v1(sx_(t), sy_(t), sz_(t));
  t += .1;
  actual_distance = 0.0;

  while (t < (max_t_ - .2) && actual_distance < point_spacing)
  {
    Eigen::Vector3d v2(sx_(t), sy_(t), sz_(t));
    actual_distance = (v2 - v1).norm();
    t += .1;
  }
  if (t > max_t_)
    t = max_t_;

  Eigen::Quaterniond Q(sqw_(t), sqx_(t), sqy_(t), sqz_(t));
  Q.normalize();

  P.position.x = sx_(t);
  P.position.y = sy_(t);
  P.position.z = sz_(t);
  P.orientation.x = Q.x();
  P.orientation.y = Q.y();
  P.orientation.z = Q.z();
  P.orientation.w = Q.w();
  return (P);
}  // end getPoseAtDistance

}  // namespace smooth_pose_traj
