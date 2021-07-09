#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"

#include <visualization_msgs/Marker.h>
#include <geometric_shapes/shape_to_marker.h>

#include "ros/ros.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;


    std::vector<pcl::Vertices> polys;
//    pcl::Vertices v1, v2;
//    v1.vertices.push_back(0);
//    v1.vertices.push_back(1);
//    v1.vertices.push_back(2);
//    v2.vertices.push_back(2);
//    v2.vertices.push_back(3);
//    v2.vertices.push_back(0);

//    polys.push_back(v1);
//    polys.push_back(v2);

//    pcl::PointXYZ pt0, pt1, pt2, pt3;
//    pt0.x=0; pt0.y=0; pt0.z=0;
//    pt1.x=1; pt1.y=0; pt1.z=0;
//    pt2.x=0; pt2.y=1; pt2.z=0;
//    pt3.x=1; pt3.y=1; pt3.z=0;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
//    cloud_->push_back(pt0);
//    cloud_->push_back(pt1);
//    cloud_->push_back(pt2);
//    cloud_->push_back(pt3);

    pcl::PolygonMesh mesh;

////    sensor_msgs::PointCloud2 sensor_cloud_;
//    ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1);
////    pcl::toROSMsg(*cloud_, sensor_cloud_);
////    cloud_->header.stamp = ros::Time::now();
//    cloud_->header.frame_id = "map";
//    cloud_pub.publish(*cloud_);
//    ros::Rate rate(0.2);
//    while (ros::ok()){
////      cloud_.header.stamp = ros::Time::now();
//      cloud_pub.publish(*cloud_);
//      rate.sleep();
//      ROS_INFO("publish");
//    }


    std::string in_path;
    std::string out_path;
    int rows, cols, sample_size;
    nh.getParam("/pcl_pc_to_mesh/in_path", in_path);
    nh.getParam("/pcl_pc_to_mesh/out_path", out_path);
    nh.getParam("/pcl_pc_to_mesh/rows", rows);
    nh.getParam("/pcl_pc_to_mesh/cols", cols);
    nh.getParam("/pcl_pc_to_mesh/sample_size", sample_size);
    ROS_INFO_STREAM(in_path);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader reader;
    reader.read(in_path, *cloud);

    if (cloud->size() != rows * cols)
    {
      ROS_ERROR("cloud.size() != rows*cols");
      return 1;
    }

    int reduced_rows = rows/sample_size;
    int reduced_cols = cols/sample_size;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> ind_map;

    ROS_INFO_STREAM("number of points = " << cloud->size());
    int index;
    int num_valid_pts = 0;
//    for (int i=0; i<cloud->size(); i++){
    for (int i=0; i<reduced_rows; i++){
      for (int j=0; j<reduced_cols ; j++){
        index = i*cols*sample_size + j*sample_size;
        pcl::PointXYZ pt = cloud->at(index);
//        ROS_INFO_STREAM("x=" << pt.x <  <", y="<<pt.y<<", z="<<pt.z);
        if (pt.x!=0 && pt.y!=0 && pt.z!=0){
          pt.x /= 1000.0;
          pt.y /= 1000.0;
          pt.z /= 1000.0;
          reduced_cloud->push_back(pt);
          ind_map.push_back(num_valid_pts);
          num_valid_pts ++;
        }
        else{
          ind_map.push_back(-1);
        }
      }
    }

    ROS_INFO_STREAM("FINISH VERTICES. Total num = " << num_valid_pts);
    int num_triangles = 0;
    int num_discarded_triangles = 0;
    for (int i=0; i<reduced_rows-1; i++){
      for (int j=0; j<reduced_cols-1; j++){
        int ind_1 = i*reduced_cols + j;
        int ind_2 = (i+1)*reduced_cols + j;
        int ind_3 = i*reduced_cols + j+1;
        if ((ind_map.at(ind_1) != -1) && (ind_map.at(ind_2) != -1) && (ind_map.at(ind_3) != -1)){
          pcl::Vertices vertices;
          vertices.vertices.push_back(ind_map.at(ind_1));
          vertices.vertices.push_back(ind_map.at(ind_2));
          vertices.vertices.push_back(ind_map.at(ind_3));
          polys.push_back(vertices);
          num_triangles++;
        }
        else {
          num_discarded_triangles++;
        }

        int ind_4 = i*reduced_cols + j+1;
        int ind_5 = (i+1)*reduced_cols + j;
        int ind_6 = (i+1)*reduced_cols + j+1;
        if ((ind_map.at(ind_4) != -1) && (ind_map.at(ind_5) != -1) && (ind_map.at(ind_6) != -1)){
          pcl::Vertices vertices;
          vertices.vertices.push_back(ind_map.at(ind_4));
          vertices.vertices.push_back(ind_map.at(ind_5));
          vertices.vertices.push_back(ind_map.at(ind_6));
          polys.push_back(vertices);
          num_triangles++;
        }
        else {
          num_discarded_triangles++;
        }
      }
    }

//    pcl::PLYWriter writer;
//    writer.write("/home/cwolfe/heat_method_ws/src/Part Meshes/small_reduced_blade_cloud_06-08.ply", *reduced_cloud);

    ROS_INFO_STREAM("finish TRIANGLES. total num = " << num_triangles << ", num_discarded = " << num_discarded_triangles);

    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*reduced_cloud, *cloud_blob);

    mesh.polygons = polys;
    mesh.cloud = *cloud_blob;

//    std::string ply_filename("/home/cwolfe/heat_method_ws/src/Part Meshes/small_reduced_blade_mesh_06-08.ply");
    pcl::io::savePLYFile(out_path, mesh);

    ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1);
    reduced_cloud->header.frame_id = "map";
    cloud_pub.publish(*reduced_cloud);
    ros::Rate rate(0.2);
    while (ros::ok()){
      cloud_pub.publish(*reduced_cloud);
      rate.sleep();
      ROS_INFO("publish");
    }

//    std::tie(mesh, geometry) = makeSurfaceMeshAndGeometry(reduced_vMat, reduced_fMat);

//    sensor_msgs::PointCloud2 cloud_;
//    ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1);
//    pcl::toROSMsg(*cloud, cloud_);
//    cloud->header.stamp = ros::Time::now();
//    cloud->header.frame_id = "map";
//    cloud_pub.publish(*cloud);
//    ros::Rate rate(0.2);
//    while (ros::ok()){
//      cloud.header.stamp = ros::Time::now();
//      cloud_pub.publish(*cloud);
//      rate.sleep();
//      ROS_INFO("publish");
//    }


//    sensor_msgs::PointCloud2 cloud_ros;
//    pcl::toROSMsg(cloud_ros, cloud);
}
