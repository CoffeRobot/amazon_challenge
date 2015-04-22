#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/PCLPointCloud2.h>
#include <memory>
#include <tf/transform_listener.h>
#include <cstdlib>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>
#include <pcl/common/common.h>

using namespace pcl;
using namespace std;

unique_ptr<pcl::visualization::PCLVisualizer> g_viewer;
PointCloud<PointXYZ>::Ptr g_cloud(new PointCloud<PointXYZ>);
unique_ptr<tf::TransformListener> g_listener;


pcl::PointCloud<PointXYZ>::Ptr g_model_cloud(new PointCloud<PointXYZ>);
float g_x, g_y, g_z;

bool g_loaded = false;

void alignShelf() {

  g_viewer->removePointCloud("estimated");
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // Define a translation of 2.5 meters on the x axis.
  transform(0, 3) = g_x;
  transform(1, 3) = g_y;
  transform(2, 3) = g_z;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud(*g_model_cloud, *transformed_cloud, transform);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(20);
  std::cout << "Ierations " << icp.getMaximumIterations() << std::endl;
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.05);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon (1);


  icp.setInputCloud(transformed_cloud);
  icp.setInputTarget(g_cloud);
  pcl::PointCloud<pcl::PointXYZ> final;
  icp.align(final);
  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
      final.makeShared(), 0, 0, 255);
  g_viewer->addPointCloud<pcl::PointXYZ>(final.makeShared(), single_color,
                                         "estimated");

}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed" << std::endl;
  }
  if (event.getKeySym () == "i" && event.keyUp())
  {
    alignShelf();
    std::cout << "i was reealsed" << std::endl;
  }
}

void updateViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  g_viewer->removePointCloud("view");
  g_viewer->addPointCloud<pcl::PointXYZ>(cloud, "view");
  //
}

void createViewer() {
  g_viewer = unique_ptr<pcl::visualization::PCLVisualizer>(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  g_viewer->setBackgroundColor(0, 0, 0);
  g_viewer->addCoordinateSystem(1.0);
  g_viewer->initCameraParameters();
  //g_viewer->setCameraPosition(0,0,0,1,0,0,0,1,0);
  g_viewer->registerKeyboardCallback(keyboardEventOccurred);
}

void showShelfModel() {
  g_viewer->removePointCloud("model");

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // Define a translation of 2.5 meters on the x axis.
  transform(0, 3) = g_x;
  transform(1, 3) = g_y;
  transform(2, 3) = g_z;

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud(*g_model_cloud, *transformed_cloud, transform);


  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
      transformed_cloud, 0, 255, 0);
  g_viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, single_color, "model");
  g_viewer->updateSphere(pcl::PointXYZ(g_x, g_y, g_z), 0.1, 255, 255, 0, "shelf_frame");
}



void loadShelfModel() {
  string meshname =
      "/home/alessandro/catkin_ws/src/pr2_amazon_challenge_sim/kiva_pod/meshes/"
      "pod_medres_rot4.pcd";

  pcl::PCDReader pcd_reader;

  PointCloud<PointXYZ> cloud, transformed;
  pcd_reader.read(meshname, cloud);

  g_model_cloud = cloud.makeShared();
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

  //if (g_loaded) return;

  sensor_msgs::PointCloud2 out;
  pcl_ros::transformPointCloud("base_footprint", *msg, out, *g_listener);

  PointCloud<PointXYZ> cloud;

  pcl::fromROSMsg(out, *g_cloud);

  //cloud.header = pcl_conversions::toPCL(msg->header);

  ROS_INFO("Point cloud received!");

  updateViewer(g_cloud->makeShared());

}

void getTransform(tf::TransformListener& listener, std::string target, std::string dest, tf::StampedTransform& transform)
{
    try{
        listener.lookupTransform(target, dest, ros::Time(0), transform);

    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
    }
}


void getShelfLocation(tf::TransformListener& listener) {

  tf::StampedTransform transform;
  string target_frame = "base_footprint";
  string dest_frame = "shelf_frame";

  if(!listener.frameExists(target_frame))
  {
    ROS_ERROR("target_frame does not exist");
    std::vector<string> names;
    listener.getFrameStrings(names);
    std::cout << "Size of names " << names.size() << std::endl;
  }
  if(!listener.frameExists(dest_frame))
  {
    ROS_ERROR("dest_frame does not exist");
    std::vector<string> names;
    listener.getFrameStrings(names);
    std::cout << "Size of names " << names.size() << std::endl;
  }
  getTransform(listener, target_frame, dest_frame, transform);

  auto origin = transform.getOrigin();
  g_x = origin.x();
  g_y = origin.y();
  g_z = origin.z();



  auto rotation = transform.getRotation();
  Eigen::Quaternionf q(rotation.getW(), rotation.getX(), rotation.getY(), rotation.getZ() );
  Eigen::Vector3f e(origin.x(), origin.y(), origin.z());

  //g_viewer->removeShape("shelf_cube");
  //std::cout << "Rot: X " << rotation.getX() << " Y " << rotation.getY() << " Z " << rotation.getZ() << " W " << rotation.getW() << std::endl;
  //g_viewer->addCube(e, q, 1, 1, 1, "shelf_cube");

  // getting distance from ground
  tf::StampedTransform ground_transform;
  getTransform(listener, "base_laser_link","base_footprint",ground_transform);
  origin = ground_transform.getOrigin();
  ground_transform.getRotation();
  /*std::cout << "X " << origin.x() << " Y " << origin.y() << " Z " << origin.z() << std::endl;

  tf::StampedTransform tmp1, tmp2;
  getTransform(listener, target_frame, "base_laser_link", tmp1);
  getTransform(listener, target_frame, "base_footprint", tmp2);

  auto origin1 = tmp1.getOrigin();
  std::cout << "X1 " << origin1.x() << " Y1 " << origin1.y() << " Z1 " << origin1.z() << std::endl;
  auto origin2 = tmp2.getOrigin();
  std::cout << "X2 " << origin2.x() << " Y2 " << origin2.y() << " Z2 " << origin2.z() << std::endl;

  float dx = origin1.x() - origin2.x();
  float dy = origin1.y() - origin2.y();
  float dz = origin1.z() - origin2.z();

  std::cout << "DX " << dx  << " DY " << dy << " DZ " << dz  << std::endl;*/

  g_z -= std::abs(origin.z());
  g_x += 0.43;


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_listener");

  createViewer();
  loadShelfModel();


  g_x = 0;
  g_y = 0;
  g_z = 0;

  g_viewer->addSphere(pcl::PointXYZ(g_x, g_y, g_z), 1, 255, 255, 0, "shelf_frame");
  // alignShelf();
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/head_mount_kinect/depth/points", 1, pointCloudCallback);

  tf::TransformListener listener;
  g_listener = unique_ptr<tf::TransformListener>(new tf::TransformListener());
  //listener.waitForTransform("ir?kinect", "basefooprint",

  while (!g_viewer->wasStopped()) {
    getShelfLocation(listener);
    showShelfModel();
    ros::spinOnce();
    g_viewer->spinOnce(1);
  }

  cout << "Or here" << endl;
  ros::shutdown();
  return 0;
}
