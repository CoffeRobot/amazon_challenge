#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <mutex>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <vision/ReceiveCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sstream>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <random>
#include <pcl/common/pca.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl/common/common.h>
#include <atomic>
#include <../include/objectsegmentation.h>

using namespace std;

namespace amazon_challenge {

class CloudSegmenter {

 public:
  CloudSegmenter() : m_nh() {
    m_service_server = m_nh.advertiseService(
        "receive_point_cloud", &CloudSegmenter::receivePointCloud, this);
    m_segmentation_pub =
        m_nh.advertise<sensor_msgs::PointCloud2>("segmentation_cloud", 1);
    m_marker_pub =
        m_nh.advertise<visualization_msgs::Marker>("cluster_markers", 1);
    m_segmentation_request = false;
  };

  bool receivePointCloud(vision::ReceiveCloud::Request &req,
                         vision::ReceiveCloud::Response &res) {
    stringstream ss;
    ss << "Cloud to segment received: points ";

    m_mutex.lock();
    m_cloud.clear();
    pcl::fromROSMsg(req.cloud, m_cloud);
    ss << m_cloud.points.size();
    ROS_INFO(ss.str().c_str());
    res.result = true;
    m_mutex.unlock();
    m_segmentation_request = true;
    pcl::copyPointCloud(m_cloud, m_cloud_colour);

    pcl::io::savePCDFileASCII("test_pcd.pcd", m_cloud);

    return true;
  }

  void segmentCloud() {
    if (m_cloud.points.size() == 0) return;

    if (m_segmentation_request) {
      pcl::PointCloud<pcl::PointXYZ> plane_cloud, filter_cloud;
      // ROS_INFO("Plane segmentation");
      // findBinPlane(m_cloud, plane_cloud, filter_cloud);
      // ROS_INFO("Object segmentation");
      vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
      ObjectSegmentation os;
      os.clusterComponentsEuclidean(m_cloud, clusters);
      m_found_clusters = clusters;

      stringstream ss;
      ss << "SEG: num clusters found: " << m_found_clusters.size();
      ROS_INFO(ss.str().c_str());

      m_segmentation_request = false;
    }
  }

  void publishMarkers() {
    default_random_engine rde;
    uniform_int_distribution<int> distribution(0, 255);

    for (auto i = 0; i < m_found_clusters.size(); i++) {
      stringstream ss;
      ss << "c" << i;
      visualization_msgs::Marker m =
          mark_cluster(m_found_clusters[i], ss.str(), i, distribution(rde),
                       distribution(rde), distribution(rde));
      m_marker_pub.publish(m);
    }
    if (m_found_clusters.size() > 0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
      int cloud_size = 0;
      for (auto i = 0; i < m_found_clusters.size(); ++i) {
        if (m_found_clusters[i].points.size() > cloud_size) {
          cloud_size = m_found_clusters[i].points.size();
          cloud_ptr = m_found_clusters[i].makeShared();
        }
      }
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg(*cloud_ptr, cloud);
      m_segmentation_pub.publish(cloud);
    }
  }

 private:
  void publishSegmentation(const pcl::PointCloud<pcl::PointXYZRGB> &in_cloud) {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(in_cloud, cloud);
    m_segmentation_pub.publish(cloud);
  }

  visualization_msgs::Marker mark_cluster(
      pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, std::string ns, int id,
      int r, int g, int b) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr =
        cloud_cluster.makeShared();
    // compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*point_cloud_ptr, centroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*point_cloud_ptr, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
        covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3, 3>(0, 0) = eigDx.transpose();
    p2w.block<3, 1>(0, 3) = -1.f * (p2w.block<3, 3>(0, 0) * centroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*point_cloud_ptr, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag =
        0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // final transform
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal =
        eigDx * mean_diag +
        centroid.head<3>();  // findClusterOBB(cloud_cluster, position,
                             // rotation, w, h, d);

    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = tfinal.x();
    marker.pose.position.y = tfinal.y();
    marker.pose.position.z = tfinal.z();
    marker.pose.orientation.x = qfinal.x();
    marker.pose.orientation.y = qfinal.y();
    marker.pose.orientation.z = qfinal.z();
    marker.pose.orientation.w = qfinal.w();

    marker.scale.x = max_pt.x - min_pt.x;
    marker.scale.y = max_pt.y - min_pt.y;
    marker.scale.z = max_pt.z - min_pt.z;

    /*marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = (max[0] - min[0]);
    marker.scale.y = (max[1] - min[1]);
    marker.scale.z = (max[2] - min[2]);
   */

    if (marker.scale.x == 0) marker.scale.x = 0.1;

    if (marker.scale.y == 0) marker.scale.y = 0.1;

    if (marker.scale.z == 0) marker.scale.z = 0.1;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();
    //   marker.lifetime = ros::Duration(0.5);
    return marker;
  }

  ros::NodeHandle m_nh;
  ros::Publisher m_segmentation_pub, m_marker_pub;
  ros::ServiceServer m_service_server;
  tf::TransformListener m_tf_listener;
  visualization_msgs::Marker m_shelf_marker;

  std::mutex m_mutex;
  pcl::PointCloud<pcl::PointXYZ> m_cloud, m_filtered_cloud, m_plane_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> m_cloud_colour;
  vector<pcl::PointCloud<pcl::PointXYZ>> m_found_clusters;
  atomic_bool m_segmentation_request;
};
}
int main(int argc, char **argv) {

  ros::init(argc, argv, "object_segmentation_node");

  amazon_challenge::CloudSegmenter cs;

  ros::Rate r(100);
  while (ros::ok()) {
    cs.segmentCloud();
    cs.publishMarkers();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
