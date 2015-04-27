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
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/image_encodings.h>
#include <memory>
#include <chrono>
#include <sstream>
#include <mutex>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include "../include/DepthTraits.h"
#include "../include/utils.h"

// Services
#include "laser_assembler/AssembleScans2.h"

using namespace std;

namespace amazon_challenge {

class PeriodicCloudPublisher {

 public:
  PeriodicCloudPublisher()
      : m_nh(),
        m_kinect_timeout_ms(1000),
        m_kinect_last_received(chrono::system_clock::now()),
        m_first_dimage_received(false)
  {
    m_pub = m_nh.advertise<sensor_msgs::PointCloud2>("periodic_cloud", 1);

    m_client =
        m_nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

    m_timer = m_nh.createTimer(ros::Duration(5, 0),
                               &PeriodicCloudPublisher::laserCallback, this);

    m_kinect_subscriber = m_nh.subscribe<sensor_msgs::Image>(
        "/head_mount_kinect/depth/image_raw", 1,
        &PeriodicCloudPublisher::depthCallback, this);
    m_camera_info_sub = m_nh.subscribe<sensor_msgs::CameraInfo>(
        "/head_mount_kinect/rgb/camera_info", 1,
        &PeriodicCloudPublisher::cameraInfoCallback, this);

    m_tf_listener =
        unique_ptr<tf::TransformListener>(new tf::TransformListener());
  }

  void buildPointCloudMessage() {

    if (m_camera_info_msg.width == 0) return;
    if (m_last_depth_msg->width == 0) return;

    m_mutex.lock();
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(m_camera_info_msg);

    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);

    if (m_last_depth_msg->encoding ==
        sensor_msgs::image_encodings::TYPE_16UC1) {
      depthToCloud<uint16_t>(m_last_depth_msg, model, *cloud_msg, 0.0);
    } else if (m_last_depth_msg->encoding ==
               sensor_msgs::image_encodings::TYPE_32FC1) {
      depthToCloud<float>(m_last_depth_msg, model, *cloud_msg, 0.0);
    }
    else
    {
        ROS_WARN("Warning: unsupported depth image format!");
        return;
    }


    //cloud_msg->header.frame_id = m_camera_info_msg.header.frame_id;
    cloud_msg->header.frame_id = m_last_depth_msg->header.frame_id;
    m_mutex.unlock();

    sensor_msgs::PointCloud2 out;
    pcl_ros::transformPointCloud("base_link", *cloud_msg, out, *m_tf_listener);

    pcl::fromROSMsg(out, m_kinect_cloud);
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& rgb_info_msg) {
    stringstream ss;

    auto can_lock = m_mutex.try_lock();

    if (!can_lock)
      return;
    else {
      m_camera_info_msg = *rgb_info_msg;
      m_mutex.unlock();
    }
  }

  void laserCallback(const ros::TimerEvent& e) {

    m_laser_start = chrono::system_clock::now();
    // We don't want to build a cloud the first callback, since we we
    //   don't have a start and end time yet
    if (m_first_time) {
      m_first_time = false;
      return;
    }

    // Populate our service request based on our timer callback times
    laser_assembler::AssembleScans2 srv;
    srv.request.begin = e.last_real;
    srv.request.end = e.current_real;

    // Make the service call
    if (m_client.call(srv)) {  

      sensor_msgs::PointCloud2 msg = srv.response.cloud;

      ROS_INFO("Tilt scanner cloud received");
      if(!m_first_dimage_received)
      {
          ROS_INFO("Kinect depth not available yet");
          return;
      }
      pcl::fromROSMsg(msg, m_laser_cloud);


      buildPointCloudMessage();

      pcl::PointCloud<pcl::PointXYZ> out_cloud;
      out_cloud = m_laser_cloud;

      out_cloud += m_kinect_cloud;

      sensor_msgs::PointCloud2 out_msg;
      pcl::toROSMsg(out_cloud, out_msg);
      m_pub.publish(out_msg);

      ROS_INFO("Aggregated cloud published");

      // pub_.publish(srv.response.cloud);
    } else {
      ROS_ERROR("Error making service call\n");
    }
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {

    auto elapsed = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now() - m_kinect_last_received).count();

    if (static_cast<float>(elapsed) < m_kinect_timeout_ms) return;
    m_kinect_last_received = chrono::system_clock::now();

    ROS_INFO("Kinect depth received");

    auto can_lock = m_mutex.try_lock();

    if (can_lock) {
      m_last_depth_msg = depth_msg;
      if(!m_first_dimage_received)
          m_first_dimage_received = true;
      m_mutex.unlock();
    } else
      return;
  }

 private:
  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::ServiceClient m_client;
  ros::Subscriber m_kinect_subscriber, m_camera_info_sub;
  ros::Timer m_timer;
  bool m_first_time;
  bool m_first_dimage_received;
  unique_ptr<tf::TransformListener> m_tf_listener;

  chrono::time_point<chrono::system_clock> m_kinect_start, m_kinect_end,
      m_laser_start, m_laser_end;
  chrono::time_point<chrono::system_clock> m_kinect_last_received;
  float m_kinect_timeout_ms;

  pcl::PointCloud<pcl::PointXYZ> m_kinect_cloud;
  pcl::PointCloud<pcl::PointXYZ> m_laser_cloud;

  sensor_msgs::CameraInfo m_camera_info_msg;
  sensor_msgs::ImageConstPtr m_last_depth_msg;
  std::mutex m_mutex;
};

}  // end namepace

int main(int argc, char** argv) {
  ROS_INFO("Init viewer!");

  ROS_INFO("Init listener!");
  ros::init(argc, argv, "composite_cloud_builder");
  // ros::NodeHandle nh;
  // LaserScannerListener scanner_listener(nh, viewer);

  amazon_challenge::PeriodicCloudPublisher aggregated_cloud_publisher;

  ros::Rate r(100);
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }



  return 0;
}
