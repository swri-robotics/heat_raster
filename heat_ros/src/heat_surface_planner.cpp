#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <heat_ros/heat_surface_planner.hpp>
#define DEBUG_CUT_AXIS 0
namespace heat
{
void HeatSurfacePlanner::planPaths(const shape_msgs::Mesh& mesh,
                                   const std::vector<int>& source_indices,
                                   std::vector<geometry_msgs::PoseArray>& paths)
{
  std::vector<int> local_source_indices;
  for (int i = 0; i < (int)source_indices.size(); i++)
    local_source_indices.push_back(source_indices[i]);

  hmTriMeshInitialize(&surface_);

  // convert Mesh to hmTriMesh
  surface_.nVertices = mesh.vertices.size();
  surface_.nFaces = mesh.triangles.size();

  // allocate
  surface_.vertices = (double*)malloc(surface_.nVertices * 3 * sizeof(double));
  surface_.texCoords = (double*)malloc(surface_.nVertices * 2 * sizeof(double));
  surface_.faces = (size_t*)malloc(surface_.nFaces * 3 * sizeof(size_t));

  // copy vertices and faces
  double* v = surface_.vertices;
  for (int i = 0; i < surface_.nVertices; i++)  // copy each vertex
  {
    v[0] = mesh.vertices[i].x;
    v[1] = mesh.vertices[i].y;
    v[2] = mesh.vertices[i].z;
    v += 3;
  }  // end copy each vertex

  size_t* f = surface_.faces;
  for (int i = 0; i < surface_.nFaces; i++)  // copy each triangle
  {
    f[0] = mesh.triangles[i].vertex_indices[0];
    f[1] = mesh.triangles[i].vertex_indices[1];
    f[2] = mesh.triangles[i].vertex_indices[2];
    f += 3;
  }  // end copy each triangle
  distance_.surface = &surface_;

  /* set time for heat flow */
  hmTriDistanceEstimateTime(&distance_);

  if (smoothness_ > 0.0)
  {
    distance_.time *= smoothness_;
  }

  /* specify boundary conditions */
  if (boundaryConditions_ > 0.)
  {
    hmTriDistanceSetBoundaryConditions(&distance_, boundaryConditions_);
  }

  /* specify verbosity */
  distance_.verbose = verbose_;

  /* compute distance */
  hmTriDistanceBuild(&distance_);

  int nv = distance_.surface->nVertices;
  hmClearArrayDouble(distance_.isSource.values, nv, 0.);
  if (local_source_indices.size() == 0)
  {
    Eigen::Vector3d N;
    double D;
    getCuttingPlane(mesh, config_.raster_rot_offset, N, D);

    // Create a new sources vector that contains all points within a small distance from this plane
    int num_source_verts = 0;
    for (int i = 0; i < (int)mesh.vertices.size(); i++)
    {
      double d = N.x() * mesh.vertices[i].x + N.y() * mesh.vertices[i].y + N.z() * mesh.vertices[i].z - D;
      if (fabs(d) < config_.raster_spacing / 7.0)
      {
        num_source_verts++;
        distance_.isSource.values[i] = 1.0;
        local_source_indices.push_back(i);
      }
    }
    if (DEBUG_CUT_AXIS)
      printf("found %d source vertices\n", num_source_verts);
  }
  else
  {                                                        // use provided sources
    for (int i = 0; i < local_source_indices.size(); i++)  // set sources
    {
      int n = local_source_indices[i];
      if (n >= nv)
      {
        printf("Source index %d is set to %d but we only have %d vertices in mesh\n", i, n, nv);
      }
      else
      {
        distance_.isSource.values[n] = 1.;
      }
    }  // end setting sources
  }
  // calculate the distances
  hmTriDistanceUpdate(&distance_);
  hmTriHeatPaths THP(&distance_, config_.raster_spacing);

  THP.compute(&distance_, local_source_indices);

  for (int i = 0; i < (int)THP.pose_arrays_.size(); i++)
  {
    geometry_msgs::PoseArray PA;
    for (int j = 0; j < (int)THP.pose_arrays_[i].size(); j++)
    {
      geometry_msgs::Pose P;
      P.position.x = THP.pose_arrays_[i][j].x;
      P.position.y = THP.pose_arrays_[i][j].y;
      P.position.z = THP.pose_arrays_[i][j].z;
      P.orientation.w = THP.pose_arrays_[i][j].qw;
      P.orientation.x = THP.pose_arrays_[i][j].qx;
      P.orientation.y = THP.pose_arrays_[i][j].qy;
      P.orientation.z = THP.pose_arrays_[i][j].qz;
      PA.poses.push_back(P);
    }
    paths.push_back(PA);
  }

  /* deallocate data structures*/
  // TODO there are known memory leaks fix them
  hmTriMeshDestroy(&surface_);
  hmTriDistanceDestroy(&distance_);
  hmContextDestroy(&context_);
}  // end of plan_paths()

bool HeatSurfacePlanner::getCellCentroidData(const shape_msgs::Mesh& mesh,
                                             const int id,
                                             Eigen::Vector3d& center,
                                             double& area)
{
  geometry_msgs::Point pt1 = mesh.vertices[(mesh.triangles[id].vertex_indices[0])];
  geometry_msgs::Point pt2 = mesh.vertices[(mesh.triangles[id].vertex_indices[1])];
  geometry_msgs::Point pt3 = mesh.vertices[(mesh.triangles[id].vertex_indices[2])];
  Eigen::Vector3d va(pt1.x - pt2.x, pt1.y - pt2.y, pt1.z - pt2.z);
  Eigen::Vector3d vb(pt2.x - pt3.x, pt2.y - pt3.y, pt2.z - pt3.z);
  Eigen::Vector3d vc(pt3.x - pt1.x, pt3.y - pt1.y, pt3.z - pt1.z);
  double a = va.norm();
  double b = vb.norm();
  double c = vc.norm();
  double s = (a + b + c) / 2.0;
  area = sqrt(s * (s - a) * (s - b) * (s - c));
  center.x() = (pt1.x + pt2.x + pt3.x) / 3.0;
  center.y() = (pt1.y + pt2.y + pt3.y) / 3.0;
  center.z() = (pt1.z + pt2.z + pt3.z) / 3.0;
  return (true);
}  // end of getCellCentroidData()

bool HeatSurfacePlanner::getCuttingPlane(const shape_msgs::Mesh& mesh,
                                         const double raster_angle,
                                         Eigen::Vector3d& N,
                                         double& D)
{
  // TODO put this in its own function
  // find center of mass of mesh
  Eigen::Vector3d C;
  double A;
  std::vector<Eigen::Vector3d> centers;
  std::vector<double> areas;
  for (int i = 0; i < mesh.triangles.size(); i++)
  {
    Eigen::Vector3d Ci;
    double Ai;
    getCellCentroidData(mesh, i, Ci, Ai);
    C += Ci * Ai;
    A += Ai;
    centers.push_back(Ci);
    areas.push_back(Ai);
  }
  C = C / A;
  Eigen::Matrix<double, 3, 3> Inertia;
  for (int i = 0; i < mesh.triangles.size(); i++)
  {
    double xk = centers[i].x() - C.x();
    double yk = centers[i].y() - C.y();
    double zk = centers[i].z() - C.z();
    Inertia(0, 0) += areas[i] * (yk * yk + zk * zk);
    Inertia(1, 1) += areas[i] * (xk * xk + zk * zk);
    Inertia(2, 2) += areas[i] * (xk * xk + yk * yk);
    Inertia(0, 1) -= areas[i] * xk * yk;
    Inertia(0, 2) -= areas[i] * xk * zk;
    Inertia(1, 2) -= areas[i] * yk * zk;
  }
  Inertia(1, 0) = Inertia(0, 1);
  Inertia(2, 0) = Inertia(0, 2);
  Inertia(2, 1) = Inertia(2, 1);
  Eigen::JacobiSVD<Eigen::Matrix3d> SVD(Inertia, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U = SVD.matrixU();

  // These two axes and the center defines the plane
  Eigen::Vector3d max_axis(U(0, 0), U(1, 0), U(2, 0));
  Eigen::Vector3d mid_axis(U(0, 1), U(1, 1), U(2, 1));
  Eigen::Vector3d min_axis(U(0, 2), U(1, 2), U(2, 2));
  if (DEBUG_CUT_AXIS)
  {
    printf("Max_axis %lf %lf %lf\n", max_axis[0], max_axis[1], max_axis[2]);
    printf("Mid_axis %lf %lf %lf\n", mid_axis[0], mid_axis[1], mid_axis[2]);
    printf("Min_axis %lf %lf %lf\n", min_axis[0], min_axis[1], min_axis[2]);
    printf("raster angle = %lf\n", raster_angle);
  }
  Eigen::Quaterniond rot(Eigen::AngleAxisd(raster_angle, min_axis));
  rot.normalize();

  N = rot.toRotationMatrix() * mid_axis;

  // equation of a plane through a center point with a given normal is
  // nx(X-cx) + ny(Y-cy) + nz(Z-cz) =0
  // nxX + nyY + nzZ = nxcx + nycy + nzcz = d where d = nxcx + nycy + nzcz
  // Distance from point to plan D = nxX + nyY + nzZ -d
  D = N.x() * C.x() + N.y() * C.y() + N.z() * C.z();
  if (DEBUG_CUT_AXIS)
    printf("Plane equation %6.3lfx %6.3lfy %6.3lfz = %6.3lf\n", N[0], N[1], N[2], D);
  return (true);
}
}  // end of namespace heat
