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

using namespace std;

namespace amazon_challenge {

class CloudSegmenter {

 public:
  CloudSegmenter() : m_nh() {
    m_service_server = m_nh.advertiseService(
        "receive_point_cloud", &CloudSegmenter::receivePointCloud, this);
    m_segmentation_pub =
        m_nh.advertise<sensor_msgs::PointCloud2>("segmentation_cloud", 1);
    m_marker_pub = m_nh.advertise<visualization_msgs::Marker>("cluster_markers", 1);
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


    pcl::io::savePCDFileASCII ("test_pcd.pcd", m_cloud);

    return true;
  }

  void segmentCloud() {
    if (m_cloud.points.size() == 0) return;

    if(m_segmentation_request)
    {
        pcl::PointCloud<pcl::PointXYZ> plane_cloud, filter_cloud;
        // ROS_INFO("Plane segmentation");
        //findBinPlane(m_cloud, plane_cloud, filter_cloud);
        // ROS_INFO("Object segmentation");
        vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
        clusterComponents(m_cloud, clusters);


        m_found_clusters = clusters;

        stringstream ss;
        ss << "SEG: num clusters found: " << m_found_clusters.size();
        ROS_INFO(ss.str().c_str());

        m_segmentation_request = false;
    }

  }

  void publishMarkers()
  {
    default_random_engine rde;
    uniform_int_distribution<int> distribution(0, 255);

    for(auto i = 0; i < m_found_clusters.size(); i++)
    {
        stringstream ss;
        ss << "c" << i;
        visualization_msgs::Marker m = mark_cluster(m_found_clusters[i], ss.str(), i,
                                                    distribution(rde),
                                                    distribution(rde),
                                                    distribution(rde));
        m_marker_pub.publish(m);
    }
    if(m_found_clusters.size() > 0)
    {
        sensor_msgs::PointCloud2 cloud;
        pcl::toROSMsg(m_found_clusters[0], cloud);
        m_segmentation_pub.publish(cloud);
    }
  }

 private:
  void publishSegmentation(const pcl::PointCloud<pcl::PointXYZRGB> &in_cloud) {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(in_cloud, cloud);
    m_segmentation_pub.publish(cloud);
  }

  void findBinPlane(pcl::PointCloud<pcl::PointXYZ>& in_cloud,
                    pcl::PointCloud<pcl::PointXYZ>& plane_cloud,
                    pcl::PointCloud<pcl::PointXYZ>& filtered_cloud) {

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = in_cloud.makeShared();
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

    if(inliers->indices.size() > 0)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(plane_cloud);

        extract.setInputCloud(cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(filtered_cloud);
        ROS_INFO("SEG: plane found");
    }
    else
    {
        filtered_cloud = in_cloud;
        ROS_INFO("SEG: no plane found");

    }


  }

  void clusterComponents(pcl::PointCloud<pcl::PointXYZ> &in_cloud,
                         vector<pcl::PointCloud<pcl::PointXYZ>>& clusters) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in_cloud.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.005);  // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in_cloud.makeShared());
    ec.extract(cluster_indices);

    //default_random_engine rde;
    //uniform_int_distribution<int> distribution(0, 255);

    //pcl::copyPointCloud(m_cloud, out_cloud);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      //auto r = distribution(rde);
      //auto g = distribution(rde);
      //auto b = distribution(rde);

      pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
      tmp_cloud.header = in_cloud.header;

      for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

        //out_cloud.points[*pit].r = r;
        //out_cloud.points[*pit].r = g;
        //out_cloud.points[*pit].r = b;

        auto pt = in_cloud.points[*pit];
        pcl::PointXYZ p(pt.x, pt.y, pt.z);

        tmp_cloud.push_back(p);
      }


      //stringstream ss;
      //ss << "Cloud" << m_found_clusters.size();
      //visualization_msgs::Marker tmp_marker =
      //    mark_cluster(tmp_cloud, ss.str(), m_found_clusters.size(), r, g, b);
      //m_marker_pub.publish(tmp_marker);
      clusters.push_back(tmp_cloud);
    }
  }

  void findClusterOBB(pcl::PointCloud<pcl::PointXYZ> &cloud_cluster,
                      Eigen::Vector3f& position,
                      Eigen::Quaternionf& rotation,
                      float& w, float& h, float& d) {
    for (auto i = 0; i < m_found_clusters.size(); ++i) {
      pcl::PCA<pcl::PointXYZ> pca;
      pcl::PointCloud<pcl::PointXYZ> proj;

      pca.setInputCloud(m_found_clusters[i].makeShared());
      pca.project(m_found_clusters[i], proj);

      pcl::PointXYZ proj_min, proj_max;
      pcl::getMinMax3D(proj, proj_min, proj_max);

      rotation = Eigen::Quaternionf(pca.getEigenVectors());

      Eigen::Vector4f t = pca.getMean();
      position = Eigen::Vector3f(t.x(), t.y(), t.z());

      float width = fabs(proj_max.x - proj_min.x);
      float height = fabs(proj_max.y - proj_min.y);
      float depth = fabs(proj_max.z - proj_min.z);
    }
  }


  visualization_msgs::Marker mark_cluster(
      pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, std::string ns, int id,
      int r, int g, int b) {
    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;

    pcl::compute3DCentroid(cloud_cluster, centroid);
    pcl::getMinMax3D(cloud_cluster, min, max);

    Eigen::Vector3f position;
    Eigen::Quaternionf rotation;
    float w, h, d;
    findClusterOBB(cloud_cluster, position, rotation, w, h, d);

    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.header.stamp = ros::Time::now();

    marker.ns = ns;
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    //marker.pose.position.x = position.x();
    //marker.pose.position.y = position.y();
    //marker.pose.position.z = position.z();
    //marker.pose.orientation.x = rotation.x();
    //marker.pose.orientation.y = rotation.y();
    //marker.pose.orientation.z = rotation.z();
    //marker.pose.orientation.w = rotation.w();

    marker.scale.x = w;
    marker.scale.y = h;
    marker.scale.z = d;


    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = (max[0] - min[0]);
    marker.scale.y = (max[1] - min[1]);
    marker.scale.z = (max[2] - min[2]);

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
