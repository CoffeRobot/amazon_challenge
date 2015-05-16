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
#include <image_geometry/pinhole_camera_model.h>
#include <random>
#include <pcl/common/pca.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl/common/common.h>
#include <atomic>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include "../include/objectsegmentation.h"
#include "../include/utils.h"
#include "../include/objectrecognition.h"

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
  CloudSegmenter() : m_nh(), m_object_recognition() {
    m_service_server = m_nh.advertiseService(
        "receive_point_cloud", &CloudSegmenter::receivePointCloud, this);
    m_segmentation_pub =
        m_nh.advertise<sensor_msgs::PointCloud2>("segmentation_cloud", 1);
    m_marker_pub =
        m_nh.advertise<visualization_msgs::Marker>("cluster_markers", 1);
    m_image_pub = m_nh.advertise<sensor_msgs::Image>("seg_box_projection", 1);

    m_task_subscriber = m_nh.subscribe("/amazon_next_task", 10,
                                       &CloudSegmenter::nextTaskCallback, this);
    m_bin_item_subscriber = m_nh.subscribe(
        "/amazon_bin_items", 10, &CloudSegmenter::binItemsCallback, this);
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
      m_camera_info = req.rgb_info;
      sensor_msgs::Image img = req.rgb_img;

      sensor_msgs::ImageConstPtr img_ptr =
          boost::make_shared<sensor_msgs::Image>(req.rgb_img);

      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr =
            cv_bridge::toCvShare(img_ptr, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
      }
      m_rgb_image = cv_ptr->image;

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
    string separator = ",]";
    s[s.length() - 1] = separator[0];

    for (auto i = 1; i < msg->data.length(); i++) {
      char c = s[i];
      if (c != separator[0] && !num_items_done)
        num_items_s += c;
      else if (c == separator[0] && !num_items_done) {
        num_items_done = true;
      } else if (c != separator[0] && num_items_done)
        item += c;
      else if (c == separator[0] && num_items_done) {
        items.push_back(item);
        // ROS_INFO(item.c_str());
        item = "";
      } else if (c == separator[1] && num_items_done) {
        items.push_back(item);
        // ROS_INFO(item.c_str());
        item = "";
      }
    }

    // ROS_INFO(item.c_str());
    // items.push_back(item);

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

    // filterClustersbySize();

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

    projectBoundingBoxToImage();

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

  void publishImage() {
    if (m_box_image.empty()) return;

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                                   m_box_image).toImageMsg();
    m_image_pub.publish(msg);
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

  void projectBoundingBoxToImage() {

    m_rgb_image.copyTo(m_box_image);
    std::vector<cv::Rect> cluster_bbox;

    for (auto i = 0; i < m_is_valid_cluster.size(); ++i) {
      stringstream cluster_name;



      if (m_is_valid_cluster[i] != SEG_TYPE::VALID) continue;

      pcl::PointCloud<pcl::PointXYZ> &cloud = m_found_clusters[i];
      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D(cloud, min_pt, max_pt);
      float cx = max_pt.x - min_pt.x;
      float cy = max_pt.y - min_pt.y;
      float cz = max_pt.z - min_pt.z;

      SegmentPose &pose = m_cluster_pose[i];
      tf::Vector3 &centroid = m_cluster_pose[i].centroid;
      tf::Quaternion &rotation = m_cluster_pose[i].rotation;

      float min_x = centroid.x() - pose.width / 2.0;
      float min_y = centroid.y() - pose.height / 2.0;
      float min_z = centroid.z() - pose.depth / 2.0;
      float max_x = centroid.x() + pose.width / 2.0;
      float max_y = centroid.y() + pose.height / 2.0;
      float max_z = centroid.z() + pose.depth / 2.0;

      pcl::PointCloud<pcl::PointXYZ> tmp_cloud, transformed;
      tmp_cloud.header.frame_id = "base_footprint";

      tmp_cloud.points.push_back(pcl::PointXYZ(min_x, min_y, min_z));
      tmp_cloud.points.push_back(pcl::PointXYZ(min_x, min_y, max_z));
      tmp_cloud.points.push_back(pcl::PointXYZ(min_x, max_y, max_z));
      tmp_cloud.points.push_back(pcl::PointXYZ(min_x, max_y, min_z));
      tmp_cloud.points.push_back(pcl::PointXYZ(max_x, min_y, min_z));
      tmp_cloud.points.push_back(pcl::PointXYZ(max_x, min_y, max_z));
      tmp_cloud.points.push_back(pcl::PointXYZ(max_x, max_y, max_z));
      tmp_cloud.points.push_back(pcl::PointXYZ(max_x, max_y, min_z));

      pcl_ros::transformPointCloud("head_mount_kinect_rgb_optical_frame",
                                   tmp_cloud, transformed, m_tf_listener);

      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(m_camera_info);

      //ss << "camera " << model.fx() << " " << model.fy() << "\n";

      float min_u = numeric_limits<float>::max();
      float min_v = numeric_limits<float>::max();
      float max_u = 0;
      float max_v = 0;

      double unit_scaling = DepthTraits<float>::fromMeters(float(1));

      for (auto pt : transformed.points) {

        float u = ((pt.x * model.fx()) / pt.z) + model.cx();
        float v = ((pt.y * model.fy()) / pt.z) + model.cy();

        if (u > max_u) max_u = u;
        if (u < min_u) min_u = u;
        if (v > max_v) max_v = v;
        if (v < min_v) min_v = v;

        //ss << pt.x << " " << pt.y << " " << pt.z << " " << u << " " << v
        //   << "\n";
      }

      //ss << "Rect:" << min_u << " " << max_u << " " << min_v << " " << max_v;


      cv::Rect r(cv::Point2d(min_u, min_v), cv::Point2d(max_u, max_v));

      cv::rectangle(m_box_image, r, cv::Scalar(0, 255, 0), 2);

      cluster_bbox.push_back(r);
    }

    vector<vector<float>> probs;
    m_object_recognition.classifyClusters(m_rgb_image, m_bin_items,
                                          cluster_bbox, probs);

    stringstream ss;

    for(auto i = 0; i < m_bin_items.size(); ++i)
    {
        ss  << "\n" << m_bin_items[i] << ": ";

        float max_prob = 0;
        cv::Point2d p;
        string obj_name;
        int obj_id;
        for(auto j = 0; j < probs[i].size(); ++j)
        {
            auto val = probs[i][j];
            if(val > max_prob)
            {
               cv:: Rect r = cluster_bbox[j];
               p = r.tl();
               obj_name = m_bin_items[j];
               max_prob = val;
               obj_id = j;
            }


            ss << probs[i][j] << " ";

        }

        stringstream objss;
        objss << obj_id << "->" << max_prob;
        cv::putText( m_box_image, objss.str(), p, cv::FONT_HERSHEY_SIMPLEX, 0.4,
                  cv::Scalar(0, 255, 0), 1, 3 );
        ss << "\n";
    }

    //ROS_INFO(ss.str().c_str());
  }

  ros::NodeHandle m_nh;
  ros::Publisher m_segmentation_pub, m_marker_pub, m_image_pub;
  ros::ServiceServer m_service_server;
  tf::TransformListener m_tf_listener;
  tf::TransformBroadcaster m_tf_broadcaster;
  visualization_msgs::Marker m_shelf_marker;
  ros::Subscriber m_task_subscriber;
  ros::Subscriber m_bin_item_subscriber;

  std::mutex m_mutex;
  pcl::PointCloud<pcl::PointXYZ> m_cloud, m_filtered_cloud, m_plane_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> m_cloud_colour;
  vector<pcl::PointCloud<pcl::PointXYZ>> m_found_clusters;
  vector<SegmentPose> m_cluster_pose;
  vector<int> m_is_valid_cluster;
  atomic_bool m_segmentation_request;
  atomic_bool m_segmentation_ready;

  cv::Mat m_rgb_image, m_box_image;
  sensor_msgs::CameraInfo m_camera_info;

  string m_bin_name;
  string m_obj_name;
  vector<string> m_bin_items;
  atomic_bool m_is_publishing;

  ObjectRecognition m_object_recognition;
};
}
int main(int argc, char **argv) {

  ros::init(argc, argv, "object_segmentation_node");

  amazon_challenge::CloudSegmenter cs;
  // amazon_challenge::ObjectRecognition recog;

  ros::Rate r(100);
  while (ros::ok()) {
    if (cs.needToPublish()) {
      cs.publishTFPose();
      cs.publishClusters();
      cs.publishImage();
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
