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
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <memory>
#include <tf/transform_listener.h>
#include <cstdlib>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>
#include <pcl/filters/passthrough.h>
#include <pcl/common/pca.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl/common/common.h>
#include "../include/objectsegmentation.h"

using namespace pcl;
using namespace std;

unique_ptr<pcl::visualization::PCLVisualizer> g_viewer;
PointCloud<PointXYZ>::Ptr g_cloud(new PointCloud<PointXYZ>);
unique_ptr<tf::TransformListener> g_listener;
amazon_challenge::ObjectSegmentation segmentation;
default_random_engine rde;
uniform_int_distribution<int> distribution(0, 255);

pcl::PointCloud<PointXYZ>::Ptr g_model_cloud(new PointCloud<PointXYZ>);
float g_x, g_y, g_z;

bool g_loaded = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event) {
  if (event.getKeySym() == "r" && event.keyDown()) {
    std::cout << "r was pressed" << std::endl;
  }
  if (event.getKeySym() == "i" && event.keyUp()) {
    std::cout << "i was reealsed" << std::endl;
  }
}

void updateViewer(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string id,
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>& color) {
  g_viewer->removePointCloud(id);
  g_viewer->addPointCloud<pcl::PointXYZ>(cloud, color, id);
  //
}

void createViewer() {
  g_viewer = unique_ptr<pcl::visualization::PCLVisualizer>(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  g_viewer->setBackgroundColor(0, 0, 0);
  g_viewer->addCoordinateSystem(1.0);
  g_viewer->initCameraParameters();
  // g_viewer->setCameraPosition(0,0,0,1,0,0,0,1,0);
  g_viewer->registerKeyboardCallback(keyboardEventOccurred);
}

void addRotateBouindgBox(PointCloud<PointXYZ>::Ptr point_cloud_ptr, string id)
{
    // compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*point_cloud_ptr, centroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*point_cloud_ptr, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<PointXYZ> cPoints;
    pcl::transformPointCloud(*point_cloud_ptr, cPoints, p2w);

    PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // final transform
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

    g_viewer->addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z, id);
}

void addBoundingBox(PointCloud<PointXYZ>::Ptr& cloud, string id)
{
    pcl::ModelCoefficients coeff_cube;
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    coeff_cube.values.resize(10);
    coeff_cube.values[0] =
        min_pt.x() + (max_pt.x() - min_pt.x()) / 2;  // Translation along the X axis
    coeff_cube.values[1] =
        min_pt.y() + (max_pt.y() - min_pt.y()) / 2;  // Translation along the Y axis
    coeff_cube.values[2] =
        min_pt.z() + (max_pt.z() - min_pt.z()) / 2;  // Translation along the Z axis
    coeff_cube.values[3] = 0;
    coeff_cube.values[4] = 0;
    coeff_cube.values[5] = 0;
    coeff_cube.values[6] = 1;
    coeff_cube.values[7] = fabs(max_pt.x() - min_pt.x());
    coeff_cube.values[8] = fabs(max_pt.y() - min_pt.y());
    coeff_cube.values[9] = fabs(max_pt.z() - min_pt.z());
    g_viewer->addCube(coeff_cube, id);
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

  // if (g_loaded) return;

  // sensor_msgs::PointCloud2 out;
  // pcl_ros::transformPointCloud("base_footprint", *msg, out, *g_listener);

  PointCloud<PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  PointCloud<PointXYZ>::Ptr cloud_ptr = cloud.makeShared();

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_ptr);

  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, 1.3);
  pass.filter(*g_cloud);

  PointCloud<PointXYZ>::Ptr plane(new PointCloud<PointXYZ>());
  PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
  segmentation.findPlane(*g_cloud, *plane, *filtered);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> plane_color(
      plane, 0, 255, 0);
  updateViewer(plane, "plane", plane_color);

  vector<PointCloud<PointXYZ>> clusters;
  segmentation.clusterComponentsEuclidean(*filtered, clusters);

  g_viewer->removeAllShapes();

  for (auto i = 0; i < clusters.size(); ++i) {

      PointCloud<PointXYZ>::Ptr ptr = clusters[i].makeShared();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(
          ptr, distribution(rde), distribution(rde), distribution(rde));
      stringstream ss;
      ss << "cluster" << i;
      updateViewer(ptr, ss.str(), color);
      ss << "cube";
      addRotateBouindgBox(ptr, ss.str());

  }

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  // filtered_color(
  //   filtered, 0, 255, 255);
  // updateViewer(filtered, "filtered", filtered_color);

  // cloud.header = pcl_conversions::toPCL(msg->header);

  ROS_INFO("Point cloud received!");
}

void getTransform(tf::TransformListener& listener, std::string target,
                  std::string dest, tf::StampedTransform& transform) {
  try {
    listener.lookupTransform(target, dest, ros::Time(0), transform);
  }
  catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_listener");

  createViewer();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/head_mount_kinect/depth/points", 1, pointCloudCallback);

  tf::TransformListener listener;
  g_listener = unique_ptr<tf::TransformListener>(new tf::TransformListener());
  // listener.waitForTransform("ir?kinect", "basefooprint",

  while (!g_viewer->wasStopped()) {

    ros::spinOnce();
    g_viewer->spinOnce(1);
  }

  cout << "Or here" << endl;
  ros::shutdown();
  return 0;
}
