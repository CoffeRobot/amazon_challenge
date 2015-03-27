#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/PCLPointCloud2.h>
#include <memory>

using namespace pcl;
using namespace std;

unique_ptr<pcl::visualization::PCLVisualizer> g_viewer;
PointCloud<PointXYZ>::Ptr g_cloud(new PointCloud<PointXYZ>);

pcl::PointCloud<PointXYZ>::Ptr g_model_cloud(new PointCloud<PointXYZ>);

bool g_loaded = false;

void updateViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  if (g_loaded) return;
  g_viewer->removePointCloud("view");
  g_viewer->addPointCloud<pcl::PointXYZ>(cloud, "view");

  g_loaded = true;
}

void createViewer() {
  g_viewer = unique_ptr<pcl::visualization::PCLVisualizer>(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  g_viewer->setBackgroundColor(0, 0, 0);
  g_viewer->addCoordinateSystem(1.0);
  g_viewer->initCameraParameters();
}

void showShelfModel() {
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
      g_model_cloud, 0, 255, 0);
  g_viewer->addPointCloud<pcl::PointXYZ>(g_model_cloud, single_color, "model");
}

void transformShelfModel() {}

void loadShelfModel() {
  string meshname =
      "/home/alessandro/catkin_ws/src/pr2_amazon_challenge_sim/kiva_pod/meshes/"
      "pod_medres.ply";
  pcl::PLYReader reader;


  PointCloud<PointXYZ> cloud;
  reader.read(meshname, cloud);
  //reader_obj.read(meshname, cloud);
  // transform the cloud
  // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // Define a translation of 2.5 meters on the x axis.
  // transform (2,3) = 3.0;
  // transform (1,3) = -1.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  original_cloud = cloud.makeShared();
  // You can either apply transform_1 or transform_2; they are the same
  // pcl::transformPointCloud (*original_cloud, *transformed_cloud, transform);
  g_model_cloud = cloud.makeShared();

  pcl::PCDWriter pcd_writer;
  pcd_writer.write(
      "/home/alessandro/catkin_ws/src/pr2_amazon_challenge_sim/kiva_pod/meshes/"
      "pod_medres.pcd",
      cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), 1);
}

/*void alignShelf() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZ>);

  string meshname =
      "/home/alessandro/catkin_ws/src/pr2_amazon_challenge_sim/kiva_pod/meshes/"
      "pod_lowres.ply";
  pcl::PLYReader reader;

  PointCloud<PointXYZ> cloud;
  reader.read(meshname, cloud);
  cloud_in = cloud.makeShared();

  std::cout << "Saved " << cloud_in->points.size()
            << " data points to input:" << std::endl;
  for (size_t i = 0; i < cloud_in->points.size(); ++i)
    std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y
              << " " << cloud_in->points[i].z << std::endl;
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;
  for (size_t i = 0; i < cloud_in->points.size(); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_in->points.size()
            << " data points:" << std::endl;
  for (size_t i = 0; i < cloud_out->points.size(); ++i)
    std::cout << "    " << cloud_out->points[i].x << " "
              << cloud_out->points[i].y << " " << cloud_out->points[i].z
              << std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
}*/

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

  if (g_loaded) return;

  PointCloud<PointXYZ> cloud;

  pcl::fromROSMsg(*msg, *g_cloud);

  cloud.header = pcl_conversions::toPCL(msg->header);

  ROS_INFO("Point cloud received!");

  // pcl::io::savePLYFile("/home/alessandro/catkin_ws/src/pr2_amazon_challenge_sim/kiva_pod/meshes/pod_medres_kinect.ply",
  // *g_cloud);

  updateViewer(g_cloud->makeShared());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_listener");

  createViewer();
  loadShelfModel();
  showShelfModel();

  // alignShelf();

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/head_mount_kinect/depth_registered/points", 1, pointCloudCallback);

  while (!g_viewer->wasStopped()) {
    // sleep(1);
    ros::spinOnce();
    g_viewer->spinOnce(1);
  }

  cout << "Or here" << endl;
  ros::shutdown();
  return 0;
}
