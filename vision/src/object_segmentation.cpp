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

using namespace std;

namespace amazon_challenge {

class CloudSegmenter {

 public:
  CloudSegmenter() : m_nh() {
    m_service_server = m_nh.advertiseService(
        "receive_point_cloud", &CloudSegmenter::receivePointCloud, this);
    m_segmentation_pub =
        m_nh.advertise<sensor_msgs::PointCloud2>("segmentation_cloud", 1);
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
    pcl::copyPointCloud(m_cloud, m_segment_cloud);

    return true;
  }

  void segmentCloud() {
    if (m_cloud.points.size() == 0) return;

    pcl::PointCloud<pcl::PointXYZRGB> out;
    //ROS_INFO("Plane segmentation");
    findBinPlane(out);
    //ROS_INFO("Object segmentation");
    clusterComponents(out);
    //ROS_INFO("Publish segmentation");
    if(out.points.size() == 0)
    {
        ROS_WARN("SEG: bin cloud is empty");
    }
    else
        publishSegmentation(out);
  }


 private:
  void publishSegmentation(const pcl::PointCloud<pcl::PointXYZRGB> &in_cloud) {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(in_cloud, cloud);
    m_segmentation_pub.publish(cloud);
  }

  void findBinPlane(pcl::PointCloud<pcl::PointXYZRGB> &out_cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = m_cloud.makeShared();
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(0.1);

    seg.setInputCloud(cloud_ptr);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(m_plane_cloud);

    for (auto i = 0; i < inliers->indices.size(); ++i) {
      pcl::PointXYZRGB p(0, 255, 0);
      auto pt = m_cloud[inliers->indices[i]];
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      out_cloud.points.push_back(p);
    }

    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(m_filtered_cloud);
  }

  void clusterComponents(pcl::PointCloud<pcl::PointXYZRGB> &out_cloud) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(m_filtered_cloud.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02);  // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(m_filtered_cloud.makeShared());
    ec.extract(cluster_indices);

    default_random_engine rde;
    uniform_int_distribution<int> distribution(0, 255);

    pcl::copyPointCloud(m_filtered_cloud, out_cloud);

    int j = 0;
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      auto r = distribution(rde);
      auto g = distribution(rde);
      auto b = distribution(rde);
      for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

        //pcl::PointXYZRGB p(r, g, b);
        //auto pt =
        out_cloud.points[*pit].r = r;
        out_cloud.points[*pit].r = g;
        out_cloud.points[*pit].r = b;
        //p.x = pt.x;
        //p.y = pt.y;
        //p.z = pt.z;
      }
      //ss << "PointCloud representing the Cluster " << j << ": "
        // << cluster_indices[j].indices.size() << " data points." << std::endl;
      j++;
    }
    //ROS_INFO(ss.str().c_str());
  }

  ros::NodeHandle m_nh;
  ros::Publisher m_segmentation_pub;
  ros::ServiceServer m_service_server;
  tf::TransformListener m_tf_listener;
  visualization_msgs::Marker m_shelf_marker;

  std::mutex m_mutex;
  pcl::PointCloud<pcl::PointXYZ> m_cloud, m_filtered_cloud, m_plane_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> m_segment_cloud;
};
}
int main(int argc, char **argv) {

  ros::init(argc, argv, "object_segmentation_node");

  amazon_challenge::CloudSegmenter cs;

  ros::Rate r(100);
  while (ros::ok()) {
    cs.segmentCloud();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
