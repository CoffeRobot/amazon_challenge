#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <mutex>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
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
#include <../include/utils.h>
#include <amazon_challenge_bt_actions/BinTrigger.h>

using namespace std;

namespace amazon_challenge {

enum SEG_TYPE {
  VALID = 0,
  SIZE,
  HEIGHT
};

struct SegmentPose {

  tf::Vector3 centroid;
  tf::Quaternion rotation;
  float width;
  float height;
  float depth;

  SegmentPose(tf::Vector3 c, tf::Quaternion q, float w, float h, float d)
      : centroid(c), rotation(q), width(w), height(h), depth(d) {}

  SegmentPose() : centroid(), rotation(), width(0), height(0), depth(0) {}
};

class CloudSegmenter {

 public:
  CloudSegmenter() : m_nh() {
    m_service_server = m_nh.advertiseService(
        "receive_point_cloud", &CloudSegmenter::receivePointCloud, this);
    m_segmentation_pub =
        m_nh.advertise<sensor_msgs::PointCloud2>("segmentation_cloud", 1);
    m_marker_pub =
        m_nh.advertise<visualization_msgs::Marker>("cluster_markers", 1);
    m_task_subscriber = m_nh.subscribe("/amazon_next_task", 10,
                                       &CloudSegmenter::nextTaskCallback, this);
    m_bin_item_subscriber = m_nh.subscribe(
        "/amazon_bin_items", 10, &CloudSegmenter::binItemsCallback, this);

    m_task_manager_service =
        m_nh.serviceClient<amazon_challenge_bt_actions::BinTrigger>(
            "bin_trigger");

    m_segmentation_request = false;
    m_is_publishing = false;
  };

  bool receivePointCloud(vision::ReceiveCloud::Request &req,
                         vision::ReceiveCloud::Response &res) {

    if (req.stop_publish) {
      m_is_publishing = false;
      res.result = true;
      ROS_INFO("Segmentation: stop publishing!");
    } else {
      stringstream ss;
      ss << "Cloud to segment received: points ";

      m_mutex.lock();
      m_cloud.clear();
      pcl::fromROSMsg(req.cloud, m_cloud);
      ss << m_cloud.points.size();
      ROS_INFO(ss.str().c_str());

      m_mutex.unlock();
      // pcl::copyPointCloud(m_cloud, m_cloud_colour);
      m_segmentation_request = true;
      m_segmentation_ready = false;
      segmentCloud();

      m_is_publishing = true;

      res.result = true;
      return true;
    }
  }

  void nextTaskCallback(const std_msgs::String::ConstPtr &msg) {
    m_bin_name = "";
    m_obj_name = "";

    for (auto i = 1; i < msg->data.length() - 1; i++) {
      if (i < 6)
        m_bin_name += msg->data[i];
      else if (i > 6)
        m_obj_name += msg->data[i];
    }
  }

  void binItemsCallback(const std_msgs::String::ConstPtr &msg) {
    string num_items_s = "";
    vector<string> items;
    string item = "";
    bool num_items_done = false;

    string s = msg->data;
    string separator = ",";

    for (auto i = 1; i < msg->data.length() - 1; i++) {
      char c = s[i];
      if (c != separator[0] && !num_items_done)
        num_items_s += c;
      else if (c == separator[0] && !num_items_done) {
        num_items_done = true;
      } else if (c != separator[0] && num_items_done)
        item += c;
      else if (c == separator[0] && num_items_done) {
        items.push_back(item);
        item = "";
      }
    }

    items.push_back(item);

    m_bin_items = items;
  }

  void filterClustersbySize() {
    for (auto i = 0; i < m_found_clusters.size(); ++i) {
      SegmentPose &pose = m_cluster_pose[i];
      int valid_dimensions = 0;
      if (pose.width >= 0.02 && pose.width < 0.15) valid_dimensions++;
      if (pose.height >= 0.02 && pose.height < 0.15) valid_dimensions++;
      if (pose.depth >= 0.02 && pose.depth < 0.15) valid_dimensions++;

      if (valid_dimensions < 2) {
        m_is_valid_cluster[i] = SEG_TYPE::SIZE;
      }
    }
  }

  void filterClustersByHeight() {
    tf::StampedTransform transform;
    string bin_name = "shelf_" + m_bin_name;
    if (!getTimedTransform(m_tf_listener, "base_footprint", bin_name, 2,
                           transform)) {
      ROS_WARN("SEG: tf timeout!");
      return;
    }

    for (auto i = 0; i < m_cluster_pose.size(); ++i) {
      SegmentPose &pose = m_cluster_pose[i];
      float min_z = pose.centroid.z() - pose.depth / 2.0f;
      float plane_z = transform.getOrigin().z();
      if (min_z > plane_z + 0.04f) m_is_valid_cluster[i] = SEG_TYPE::HEIGHT;
    }
  }

  void segmentCloud() {

    if (m_cloud.points.size() == 0) return;

    amazon_challenge_bt_actions::BinTrigger srv;

    if (m_task_manager_service.call(srv)) {
      auto res = srv.response.message;
      stringstream ss;
      ss << "num items: " <<res.size() << "\n";
      for(auto s : res)
          ss << s << "\n";

      ROS_INFO(ss.str().c_str());
    } else {
      ROS_ERROR("Task manager node not listening");
    }

    m_found_clusters.clear();

    pcl::PointCloud<pcl::PointXYZ> plane_cloud, filter_cloud;
    vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    ObjectSegmentation os;
    // os.clusterComponentsEuclidean(m_cloud, clusters);
    os.clusterExpectedComponents(m_bin_items.size(), m_cloud, clusters);
    m_found_clusters = clusters;
    m_cluster_pose.clear();
    m_is_valid_cluster.clear();

    for (auto cluster : m_found_clusters) {

      Eigen::Vector4f centroid;
      Eigen::Quaternionf rotation;

      float w, h, d;
      os.extractPose(cluster, centroid, rotation, w, h, d);

      tf::Vector3 tf_centroid(centroid.x(), centroid.y(), centroid.z());
      tf::Quaternion tf_rotation(rotation.x(), rotation.y(), rotation.z(),
                                 rotation.w());
      m_cluster_pose.push_back(SegmentPose(tf_centroid, tf_rotation, w, h, d));
      m_is_valid_cluster.push_back(SEG_TYPE::VALID);

      m_segmentation_ready = true;
    }

    stringstream ss;
    ss << "SEG: num clusters found: " << m_found_clusters.size();

    auto count = 0;
    for (auto b : m_is_valid_cluster) {
      if (b == SEG_TYPE::VALID) count++;
    }
    ss << " valid " << count;

    filterClustersbySize();

    count = 0;
    for (auto b : m_is_valid_cluster) {
      if (b == SEG_TYPE::VALID) count++;
    }
    ss << " size filter " << count;

    filterClustersByHeight();

    count = 0;
    for (auto b : m_is_valid_cluster) {
      if (b == SEG_TYPE::VALID) count++;
    }
    ss << " height filter " << count;

    pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
    // pcl::copyPointCloud(m_cloud, m_cloud_colour);

    default_random_engine rde;
    uniform_int_distribution<int> distribution(0, 255);

    for (auto i = 0; i < m_is_valid_cluster.size(); i++) {
      if (m_is_valid_cluster[i] == SEG_TYPE::VALID) {
        pcl::PointCloud<pcl::PointXYZRGB> cluster_rgb;
        pcl::copyPointCloud(m_found_clusters[i], cluster_rgb);

        int r = distribution(rde);
        int g = distribution(rde);
        int b = distribution(rde);

        for (auto j = 0; j < cluster_rgb.points.size(); ++j) {
          pcl::PointXYZRGB &pt = cluster_rgb.points[j];
          pt.r = r;
          pt.g = g;
          pt.b = b;
        }

        cluster_cloud += cluster_rgb;
      }
    }

    m_cloud_colour = cluster_cloud;

    ROS_INFO(ss.str().c_str());

    m_segmentation_request = false;
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
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg(m_found_clusters[i], cloud);
      m_segmentation_pub.publish(cloud);
    }
    /*if (m_found_clusters.size() > 0) {
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
    }*/
  }

  void publishTFPose() {

    if (m_cluster_pose.size() < 1) return;

    int invalid = 0;
    int valid = 0;
    for (auto i = 0; i < m_is_valid_cluster.size(); ++i) {
      stringstream cluster_name;

      if (m_is_valid_cluster[i] == SEG_TYPE::SIZE) {
        cluster_name << "size" << invalid;
        invalid++;
      } else if (m_is_valid_cluster[i] == SEG_TYPE::HEIGHT) {
        cluster_name << "height" << invalid;
        invalid++;
      } else {
        if (valid < m_bin_items.size())
          cluster_name << m_bin_items[valid] << "_scan";
        else
          cluster_name << "valid" << valid << "_scan";
        valid++;
      }

      tf::Vector3 &centroid = m_cluster_pose[i].centroid;
      tf::Quaternion &rotation = m_cluster_pose[i].rotation;

      tf::Transform transform;
      transform.setOrigin(centroid);
      transform.setRotation(rotation);

      m_tf_broadcaster.sendTransform(tf::StampedTransform(
          transform, ros::Time::now(), "base_footprint", cluster_name.str()));
    }
  }

  void publishClusters() {
    if (m_cloud_colour.points.size() > 0) {
      m_cloud_colour.header.frame_id = m_cloud.header.frame_id;
      sensor_msgs::PointCloud2 cloud;
      pcl::toROSMsg(m_cloud_colour, cloud);
      m_segmentation_pub.publish(cloud);
    }
  }

  bool needToPublish() { return m_is_publishing; }

 private:
  void extractPose(const pcl::PointCloud<pcl::PointXYZ> &in_cloud,
                   Eigen::Vector4f &centroid, Eigen::Quaternionf &rotation) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr = in_cloud.makeShared();
    // compute principal direction

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
    Eigen::Vector3f mean_diag =
        0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // final transform
    Eigen::Quaternionf qfinal(eigDx);

    Eigen::Vector3f tfinal = eigDx * mean_diag + centroid.head<3>();

    rotation = qfinal;
  }

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
  tf::TransformBroadcaster m_tf_broadcaster;
  visualization_msgs::Marker m_shelf_marker;
  ros::Subscriber m_task_subscriber;
  ros::Subscriber m_bin_item_subscriber;
  ros::ServiceClient m_task_manager_service;

  std::mutex m_mutex;
  pcl::PointCloud<pcl::PointXYZ> m_cloud, m_filtered_cloud, m_plane_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> m_cloud_colour;
  vector<pcl::PointCloud<pcl::PointXYZ>> m_found_clusters;
  vector<SegmentPose> m_cluster_pose;
  vector<int> m_is_valid_cluster;
  atomic_bool m_segmentation_request;
  atomic_bool m_segmentation_ready;

  string m_bin_name;
  string m_obj_name;
  vector<string> m_bin_items;

  atomic_bool m_is_publishing;
};
}
int main(int argc, char **argv) {

  ros::init(argc, argv, "object_segmentation_node");

  amazon_challenge::CloudSegmenter cs;

  ros::Rate r(100);
  while (ros::ok()) {
    if (cs.needToPublish()) {
      cs.publishTFPose();
      cs.publishClusters();
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
