#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <memory>
#include <chrono>
#include <sstream>
#include <mutex>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <actionlib/server/simple_action_server.h>
#include "../include/DepthTraits.h"
#include "../include/utils.h"
#include <vision/BTAction.h>
#include <atomic>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

// Services
#include <laser_assembler/AssembleScans2.h>
#include <vision/ReceiveCloud.h>
#include <vision/StartAggregator.h>
#include <simtrack_nodes/UpdateValidationPointCloud.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <pr2_msgs/SetLaserTrajCmd.h>

using namespace std;

namespace amazon_challenge {

class CropTool {

 public:
  CropTool() : m_nh() {

    m_depth_it.reset(new image_transport::ImageTransport(m_nh));
    m_sub_depth.subscribe(
        *m_depth_it,
        "/head_mount_kinect/depth_registered/sw_registered/image_rect_raw", 1,
        image_transport::TransportHints("compressedDepth"));
    m_rgb_it.reset(new image_transport::ImageTransport(m_nh));
    m_sub_rgb.subscribe(*m_rgb_it, "/head_mount_kinect/rgb/image_rect_color", 1,
                        image_transport::TransportHints("compressed"));
    m_sub_rgb_info.subscribe(m_nh, "/head_mount_kinect/rgb/camera_info", 1);

    m_sync_rgbd.reset(new SynchronizerRGBD(SyncPolicyRGBD(5), m_sub_depth,
                                           m_sub_rgb, m_sub_rgb_info));
    m_sync_rgbd->registerCallback(
        boost::bind(&CropTool::rgbdCallback, this, _1, _2, _3));

    m_cloud_publisher =
        m_nh.advertise<sensor_msgs::PointCloud2>("segmentation_test_cloud", 1);
    m_shelf_pub =
        m_nh.advertise<sensor_msgs::PointCloud2>("shelf_test_cloud", 1);
  };

  void rgbdCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                    const sensor_msgs::ImageConstPtr& rgb_msg,
                    const sensor_msgs::CameraInfoConstPtr& rgb_info_msg) {

    // ROS_INFO("Received full optional kinect data :)");

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(rgb_info_msg);
    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      depthToCloud<uint16_t>(depth_msg, model, *cloud_msg, 0.0);
    } else if (depth_msg->encoding ==
               sensor_msgs::image_encodings::TYPE_32FC1) {
      depthToCloud<float>(depth_msg, model, *cloud_msg, 0.0);
    } else {
      ROS_WARN("Warning: unsupported depth image format!");
      m_mutex.unlock();
      // ROS_INFO("Mutex unlocked by cloud message");
      return;
    }

    cloud_msg->header.frame_id = rgb_msg->header.frame_id;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    pcl::copyPointCloud(cloud, m_colored_cloud);

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr =
          cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    auto w = m_colored_cloud.width;
    auto h = m_colored_cloud.height;

    const cv::Mat& img = cv_ptr->image;

    for (auto row = 0; row < h; ++row) {
      for (auto col = 0; col < w; ++col) {
        auto id = col + w * row;
        cv::Vec3b rgb = img.at<cv::Vec3b>(row, col);
        pcl::PointXYZRGB& p = m_colored_cloud.points[id];
        p.r = rgb.val[2];
        p.g = rgb.val[1];
        p.b = rgb.val[0];
      }
    }
  }

  void cropCloudBin(string bin_name,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& in,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out) {
    tf::StampedTransform transform;
    if (!getTimedTransform(m_tf_listener, "base_footprint", bin_name, 2.0f,
                           transform)) {
      ROS_WARN("Timeout in getting transform");
      return;
    }
    Eigen::Vector3f size = getBinSize(bin_name);

    auto origin = transform.getOrigin();

    float min_x = origin.x() + 0.02;
    float max_x = origin.x() + size.x() - 0.02;
    float min_y = origin.y() + 0.02;
    float max_y = origin.y() + size.y() - 0.02;
    float min_z = origin.z() + 0.01;
    float max_z = origin.z() + size.z() - 0.04f;

    cropCloud(in, min_x, max_x, min_y, max_y, min_z, max_z, out);
  }

  void cropCloudBin(string bin_name,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    tf::StampedTransform transform;
    if (!getTimedTransform(m_tf_listener, "base_footprint", bin_name, 2.0f,
                           transform)) {
      ROS_WARN("Timeout in getting transform");
      return;
    }
    Eigen::Vector3f size = getBinSize(bin_name);

    auto origin = transform.getOrigin();

    float min_x = origin.x() + 0.02;
    float max_x = origin.x() + size.x() - 0.02;
    float min_y = origin.y() + 0.02;
    float max_y = origin.y() + size.y() - 0.02;
    float min_z = origin.z() + 0.01;
    float max_z = origin.z() + size.z() - 0.04f;

    cropCloud(in, min_x, max_x, min_y, max_y, min_z, max_z, out);
  }

  void debugBin(string bin_name, pcl::PointCloud<pcl::PointXYZRGB>& out) {
    auto cloud_ptr = m_colored_cloud.makeShared();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    cropCloudBin(bin_name, cloud_ptr, cropped);
    Eigen::Vector3i color = getBinColor(bin_name);
    for (auto i = 0; i < cropped->points.size(); ++i) {
      pcl::PointXYZRGB& pt = cropped->points[i];
      pt.r = color.x();
      pt.g = color.y();
      pt.b = color.z();
    }
    out += *cropped;
  }

  void debugCropping() {
    m_d_crop_cloud.points.clear();
    debugBin("shelf_bin_A", m_d_crop_cloud);
  }

  void publishCloud() {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(m_colored_cloud, cloud);
    m_cloud_publisher.publish(cloud);
  }

  ros::NodeHandle m_nh;
  boost::shared_ptr<image_transport::ImageTransport> m_rgb_it, m_depth_it;
  image_transport::SubscriberFilter m_sub_depth, m_sub_rgb;
  message_filters::Subscriber<sensor_msgs::CameraInfo> m_sub_rgb_info;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
      SyncPolicyRGBD;
  typedef message_filters::Synchronizer<SyncPolicyRGBD> SynchronizerRGBD;
  boost::shared_ptr<SynchronizerRGBD> m_sync_rgbd;

  std::mutex m_mutex;
  sensor_msgs::CameraInfo m_camera_info_msg;

  pcl::PointCloud<pcl::PointXYZRGB> m_colored_cloud, m_d_crop_cloud;
  pcl::PointCloud<pcl::PointXYZ> m_cloud;

  ros::Publisher m_cloud_publisher, m_shelf_pub;

  tf::TransformListener m_tf_listener;
};

}  // end namespace

std::vector<std::vector<int>> array_to_matrix(int* m, int rows, int cols) {
  int i, j;
  std::vector<std::vector<int>> r;
  r.resize(rows, std::vector<int>(cols, 0));

  for (i = 0; i < rows; i++) {
    for (j = 0; j < cols; j++) r[i][j] = m[i * cols + j];
  }
  return r;
}

int main(int argc, char** argv) {

  vector<vector<double>> values;
  vector<double> v1 = {1};
  vector<double> v2 = {4};
  vector<double> v3 = {5};
  values.push_back(v1);
  values.push_back(v2);
  values.push_back(v3);
  vector<int> idx(values[0].size(),-1);

  for (auto i = 0; i < values.size(); ++i) {

      auto min_id_j = 0;
      auto min_id_k = 0;
      auto min_val = numeric_limits<double>::max();

      for (auto j = 0; j < values.size(); ++j) {
          for(auto k= 0; k < values[j].size(); ++k)
          {
                if(values[j][k] <= min_val)
                {
                    min_val = values[j][k];
                    min_id_j = j;
                    min_id_k = k;
                }
          }
      }

      cout << "min val: j " << min_id_j << " k " << min_id_k << " " << min_val << std::endl;

      for(auto w = 0; w < values[min_id_j].size(); ++w)
          values[min_id_j][w] = numeric_limits<float>::max();

      for(auto w = 0; w < values.size(); ++w)
          values[w][min_id_k] = numeric_limits<float>::max();

      idx[min_id_k] = min_id_j;

      stringstream ss;
      for(auto v : idx)
          ss << v << " ";
      cout << ss.str() << endl;
  }

  stringstream ss;
  for(auto v : idx)
      ss << v << " ";
  cout << ss.str() << endl;

  return 0;
}
