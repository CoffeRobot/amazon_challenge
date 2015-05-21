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
#include <queue>

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
      std::cout<< "object pose: " << std::endl;
      for(unsigned int i=0; i<req.obj_pos.size(); i++)
      {
          std::cout << req.obj_pos[i] << " " << std::endl;
      }

      std::cout<< "object name: " << std::endl;
      for(unsigned int i=0; i<req.obj_name.size(); i++)
      {
          std::cout << req.obj_name[i] << " " << std::endl;
      }

      auto result = segmentCloud(req.obj_pos, req.obj_name);

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

  void filterClustersByHeight(vector<SegmentPose> &poses,
                              vector<SEG_TYPE> &status) {
    tf::StampedTransform transform;
    string bin_name = "shelf_" + m_bin_name;
    if (!getTimedTransform(m_tf_listener, "base_footprint", bin_name, 2,
                           transform)) {
      ROS_WARN("SEG: tf timeout!");
      return;
    }

    for (auto i = 0; i < poses.size(); ++i) {
      SegmentPose &pose = poses[i];
      float min_z = pose.centroid.z() - pose.height / 2.0f;
      float plane_z = transform.getOrigin().z();
      std::cout << "min_z: " << min_z << ", plane_z: " << plane_z << endl; 
      if (min_z > plane_z + 0.14f) status[i] = SEG_TYPE::HEIGHT;
    }
  }

  bool checkBinStatus(int cluster_found, int bin_items, int obj_detected) {
    if (cluster_found != bin_items)
      return false;
    else if (cluster_found - obj_detected > 2)
      return false;
    else
      return true;
  }

  vector<tf::Vector3> getKnownCentroids(vector<double> &pos,
                                        vector<string> &name) {
    vector<tf::Vector3> res;
    for (int i = 0; i < name.size(); ++i) {
      res.push_back(tf::Vector3(pos[3 * i], pos[3 * i + 1], pos[3 * i + 2]));
      std::cout << res[i] << std::endl;
    }


    return res;
  }

  void assingLabels(const vector<SEG_TYPE> &status, const vector<string> &items,
                    const vector<string> &names, vector<string> &labels) {

    vector<string> labels_left;
    for (auto i = 0; i < items.size(); ++i) {
      bool found = false;
      for (int j = 0; j < names.size(); ++j) {
        if (names[j].compare(items[i]) == 0) {
          found = true;
          break;
        }
      }
      if (!found) labels_left.push_back(items[i]);
    }

    auto valid_count = 0;
    for (int i = 0; i < labels.size(); ++i) {
      if (status[i] == SEG_TYPE::VALID && labels[i].compare("") == 0 &&
          valid_count < labels_left.size()) {
        labels[i] = labels_left[valid_count];
        valid_count++;
      }
    }
  }

  void assingBestLabels(const vector<string> &items,
                        const vector<string> &names, vector<string> &labels) {

      vector<string> labels_left;
      for (auto i = 0; i < items.size(); ++i) {
        bool found = false;
        for (int j = 0; j < names.size(); ++j) {
          if (names[j].compare(items[i]) == 0) {
            found = true;
            break;
          }
        }
        if (!found) labels_left.push_back(items[i]);
      }

      auto valid_count = 0;
      for (int i = 0; i < labels.size(); ++i) {
        if (labels[i].compare("") == 0 &&
            valid_count < labels_left.size()) {
          labels[i] = labels_left[valid_count];
          valid_count++;
        }
      }


  }

  bool segmentCloud(vector<double> &pos, vector<string> &name) {

    if (m_cloud.points.size() == 0) return false;

    amazon_challenge_bt_actions::BinTrigger srv;
    vector<string> bin_items;
    /* query bin trigger to get list of objects */
    if (m_task_manager_service.call(srv)) {
      bin_items = srv.response.message;
      m_bin_items = bin_items;
    } else {
      ROS_ERROR("Task manager node not listening");
    }

    stringstream info;
    info << "known objewcts received: " << name.size() << " pos " << pos.size();
    ROS_INFO(info.str().c_str());


    //NEW: get the location of the bin
    tf::StampedTransform transform;
    string bin_name = "shelf_" + m_bin_name;
    if (!getTimedTransform(m_tf_listener, "base_footprint", bin_name, 2,
                           transform)) {
      ROS_WARN("SEG: tf timeout!");
      return false;
    }

    //NEW: calciulating the position of the bin
    auto origin = transform.getOrigin();
    Eigen::Vector3f bin_pos(origin.x(), origin.y(), origin.z());

    /* segment the constructed point cloud */
    vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    ObjectSegmentation os;
    os.clusterExpectedComponents(bin_items.size(), m_cloud, bin_pos, clusters);

    vector<SegmentPose> cluster_pose;
    vector<SEG_TYPE> cluster_status;

    /* estimate pose of each cluster */
    for (auto cluster : clusters) {

      Eigen::Vector4f centroid;
      Eigen::Quaternionf rotation;

      float w, h, d;
      os.extractPose(cluster, centroid, rotation, w, h, d);

      tf::Vector3 tf_centroid(centroid.x(), centroid.y(), centroid.z());
      tf::Quaternion tf_rotation(rotation.x(), rotation.y(), rotation.z(),
                                 rotation.w());
      cluster_pose.push_back(SegmentPose(tf_centroid, tf_rotation, w, h, d));
      cluster_status.push_back(SEG_TYPE::VALID);
      m_segmentation_ready = true;
    }

    //filterClustersByHeight(cluster_pose, cluster_status);

    ROS_INFO(toString(cluster_status).c_str());

    auto valid_count = getValid(cluster_status);

    bool status = checkBinStatus(valid_count, bin_items.size(), name.size());
    if (!status) {
      stringstream warn;
      warn << "Clustering non consistent with objects in the bin";
      ROS_WARN(warn.str().c_str());
      return false;
    }

    vector<tf::Vector3> obj_known = getKnownCentroids(pos, name);
    // vector<string> labels(clusters.size(), "");

    // calculate the best label
    vector<SegmentPose> valid_poses;
    for (auto i = 0; i < cluster_pose.size(); ++i) {
      if (cluster_status[i] == SEG_TYPE::VALID)
        valid_poses.push_back(cluster_pose[i]);
    }

    vector<vector<double>> dist_matrix(obj_known.size(),
                                       vector<double>(valid_poses.size(), 0));
    for (auto i = 0; i < obj_known.size(); ++i) {
      tf::Vector3 &pos = obj_known[i];

      for (int j = 0; j < valid_poses.size(); j++) {
        tf::Vector3 &c_pos = valid_poses[j].centroid;
        auto distance = std::sqrt(std::pow(static_cast<double>(pos.x()) -
                                               static_cast<double>(c_pos.x()),
                                           2) +
                                  std::pow(static_cast<double>(pos.y()) -
                                               static_cast<double>(c_pos.y()),
                                           2) * 2 +
                                  std::pow(static_cast<double>(pos.z()) -
                                               static_cast<double>(c_pos.z()),
                                           2));
        dist_matrix[i][j] = distance;
      }
    }
    vector<int> best_labels;
    findBestMatching(dist_matrix, best_labels);
    vector<string> labels(valid_poses.size(), "");

    stringstream ss_labels;
    ss_labels << "PAIRS:\n";
    std::cout << "best labels size " << best_labels.size() << std::endl;
//    std::cout << "name: ";
//    for (int i = 0; i < name.size(); i++){
//        std::cout << name[i] << " ";
//    }

//    std::cout << std::endl;

//    std::cout << "best_labels: ";
//    for (int i = 0; i < best_labels.size(); i++){
//        std::cout << best_labels[i] << " ";
//    }

//    std::cout << std::endl;

    int label_shift = 0;
    for (int i = 0; i < best_labels.size(); ++i) {
      if (best_labels[i] != -1) {
        labels.at(best_labels.at(i)) = name.at(i-label_shift);
      }else{
          label_shift++;
      }
      ss_labels << best_labels[i] << " ";
    }
//    std::cout << std::endl;
    ROS_INFO(ss_labels.str().c_str());
    /*    for (int j = 0; j < obj_known.size(); ++j) {
          tf::Vector3 &pos = obj_known[j];
          double min_dis = numeric_limits<double>::max();
          int obj_id = 0;
          string label = "";
          for (auto i = 0; i < clusters.size(); ++i) {
            if (cluster_status[i] == SEG_TYPE::VALID) {
              tf::Vector3 &c_pos = cluster_pose[i].centroid;
              auto distance =
                  std::sqrt(std::pow(static_cast<double>(pos.x()) -
       static_cast<double>(c_pos.x()),2) +
                            std::pow(static_cast<double>(pos.y()) -
       static_cast<double>(c_pos.y()),2) +
                            std::pow(static_cast<double>(pos.z()) -
       static_cast<double>(c_pos.z()),2));

              if (distance < min_dis && labels[i].compare("") == 0) {
                min_dis = distance;
                obj_id = i;
                label = name[j];
              }
            }
          }
          labels[obj_id] = label;
        }
    */
    //assingLabels(cluster_status, bin_items, name, labels);

    assingBestLabels(bin_items, name, labels);

    stringstream label_str;
    label_str << "LABELS:\n";
    for (auto l : labels) label_str << l << "\n";
    ROS_INFO(label_str.str().c_str());

    pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
    // pcl::copyPointCloud(m_cloud, m_cloud_colour);

    default_random_engine rde;
    uniform_int_distribution<int> distribution(0, 255);

    for (auto i = 0; i < cluster_status.size(); i++) {
      if (cluster_status[i] == SEG_TYPE::VALID) {
        pcl::PointCloud<pcl::PointXYZRGB> cluster_rgb;
        pcl::copyPointCloud(clusters[i], cluster_rgb);

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

    // assign variables for publishing
    m_cluster_labels = labels;
    m_is_valid_cluster = cluster_status;
    m_cluster_pose = valid_poses;

    // ROS_INFO(ss.str().c_str());

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
  }

  void publishTFPose() {

    if (m_cluster_pose.size() < 1) return;

    int invalid = 0;
    int valid = 0;
    for (auto i = 0; i < m_cluster_labels.size(); ++i) {
      stringstream cluster_name;

      cluster_name << m_cluster_labels[i] << "_scan";

      /*if (m_is_valid_cluster[i] == SEG_TYPE::SIZE) {
        cluster_name << "size" << invalid;
        invalid++;
      } else if (m_is_valid_cluster[i] == SEG_TYPE::HEIGHT) {
        cluster_name << "height" << invalid;
        invalid++;
      } else {
        cluster_name << m_cluster_labels[i] << "_scan";
      }*/

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

  string toString(vector<SEG_TYPE> &status) {
    stringstream ss;
    auto valid = 0;
    auto height = 0;
    auto size = 0;

    for (auto s : status) {
      if (s == SEG_TYPE::VALID) valid++;
      if (s == SEG_TYPE::HEIGHT) height++;
      if (s == SEG_TYPE::SIZE) size++;
    }

    ss << "V: " << valid << " H: " << height << " S: " << size;

    return ss.str();
  }

  int getValid(vector<SEG_TYPE> &status) {
    auto valid = 0;
    for (auto s : status)
      if (s == SEG_TYPE::VALID) valid++;
    return valid;
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
  vector<SEG_TYPE> m_is_valid_cluster;
  vector<string> m_cluster_labels;
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

  ros::Rate r(5);
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
