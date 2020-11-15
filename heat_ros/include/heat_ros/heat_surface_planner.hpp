/**
 * @file heat_surface_planner.h
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

#ifndef INCLUDE_HEAT_SURFACE_PLANNER_H
#define INCLUDE_HEAT_SURFACE_PLANNER_H

#include <shape_msgs/Mesh.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <eigen3/Eigen/Eigen>

extern "C" {
#include "libgeodesic/hmTriDistance.h"
#include "libgeodesic/hmContext.h"
#include "libgeodesic/hmUtility.h"
#include "libgeodesic/hmVectorSizeT.h"
}
#include "libgeodesic/hmHeatPath.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

namespace heat
{
struct ProcessPath
{
  std::vector<geometry_msgs::Pose> poses;
};

struct ProcessConfig
{
  double point_spacing;
  double raster_spacing;
  double tool_offset;
  double min_hole_size;
  double min_segment_size;
  bool generate_extra_rasters;
  double raster_rot_offset;
};

class HeatSurfacePlanner
{
public:
  HeatSurfacePlanner()
  {
    config_ = getDefaultConfig();
    init();
  }
  HeatSurfacePlanner(ProcessConfig& config)
  {
    config_.point_spacing = config.point_spacing;
    config_.raster_spacing = config.raster_spacing;
    config_.tool_offset = config.tool_offset;
    config_.min_hole_size = config.min_hole_size;
    config_.min_segment_size = config.min_segment_size;
    config_.raster_rot_offset = config.raster_rot_offset;
    init();
  }

  void init()
  {
    // standard parameters
    nSourceSets_ = 0;
    smoothness_ = -1.0;
    boundaryConditions_ = -1.0;
    verbose_ = 0;

    /* initialize data structures*/
    hmContextInitialize(&context_);
    hmTriMeshInitialize(&surface_);
    hmTriDistanceInitialize(&distance_);
  }

  static ProcessConfig getDefaultConfig()
  {
    ProcessConfig C;
    C.point_spacing = 0.01;
    C.raster_spacing = 0.015;
    C.tool_offset = 0.0;
    C.min_hole_size = 0.01;
    C.min_segment_size = 0.01;
    C.raster_rot_offset = 0.0;
    return (C);
  }

  ~HeatSurfacePlanner() {}

  void planPaths(const shape_msgs::Mesh&,
                 const std::vector<int>& source_indices,
                 std::vector<geometry_msgs::PoseArray>& paths);

  /** @brief from a sequence of points, create a sequence of poses(path).
   *  aligns z with local mesh normal,
   *  aligns x toward next point in sequence
   *  aligns y using right hand rule with
   **/
  bool createPoseArray(const std::vector<int>& path_indices, geometry_msgs::PoseArray& poses);

  /** @brief find the center of mass of a triangle and its area
   *  @input mesh the mesh containing vertices and triangles
   *  @input id   the index of the triangle for consideration
   *  @output center The center of mass of this triangle
   *  @output area   The area of this triangle
   **/
  bool getCellCentroidData(const shape_msgs::Mesh& mesh, const int id, Eigen::Vector3d& center, double& area);

  /** @brief find the cutting plane given the mesh and its raster angle
   *  @input mesh the mesh containing vertices and triangles
   *  @input raster_angle angle to rotate around mid-axis of principal component
   *  @output N the normal vector for the plane
   *  @output D the right hand side of the plane equation N_x X + N_y Y + N_z Z = D
   **/
  bool getCuttingPlane(const shape_msgs::Mesh& mesh, const double raster_angle, Eigen::Vector3d& N, double& D);

public:
  ProcessPath path_;
  ProcessConfig config_;

  hmContext context_;
  hmTriMesh surface_;
  hmTriDistance distance_;

  // parameters
  int nSourceSets_;
  double smoothness_;
  double boundaryConditions_;
  char verbose_ = 0;

};  // end class HeatSurfacePlanner

}  // end namespace heat
#endif  // INCLUDE_HEAT_SURFACE_PLANNER_H
