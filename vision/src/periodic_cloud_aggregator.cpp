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
  PeriodicCloudPublisher(shared_ptr<pcl::visualization::PCLVisualizer> viewer)
      : m_nh(),
        m_viewer(viewer),
        m_kinect_timeout_ms(3000),
        m_kinect_last_received(chrono::system_clock::now()) {
    m_pub = m_nh.advertise<sensor_msgs::PointCloud>("periodic_cloud", 1);

    // m_client =
    //    m_nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

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

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& rgb_info_msg) {
    stringstream ss;

    auto can_lock = m_camera_mutex.try_lock();

    if(!can_lock)
        return;
    else
    {
        m_camera_info_msg = *rgb_info_msg;
        m_camera_mutex.unlock();
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
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      // ROS_INFO("Published Cloud with laser");
      pcl::fromROSMsg(msg, pcl_cloud);

      m_viewer->removePointCloud("laser_cloud");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
          single_color(pcl_cloud.makeShared(), 255, 0, 0);
      m_viewer->addPointCloud<pcl::PointXYZ>(pcl_cloud.makeShared(),
                                             single_color, "laser_cloud");

      m_laser_end = chrono::system_clock::now();

      auto ms = chrono::duration_cast<chrono::milliseconds>(
          m_laser_end - m_laser_end).count();

      stringstream ss;
      ss << "Laser cloud with " << (uint32_t)(pcl_cloud.points.size()) << " in "
         << static_cast<float>(ms) << " ms ";

      ROS_INFO(ss.str().c_str());

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


    ROS_INFO("Depth image received");

    m_camera_mutex.lock();
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(m_camera_info_msg);
    m_camera_mutex.unlock();

    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);

    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      depthToCloud<uint16_t>(depth_msg, model, *cloud_msg, 0.0);
    } else if (depth_msg->encoding ==
               sensor_msgs::image_encodings::TYPE_32FC1) {
      depthToCloud<float>(depth_msg, model, *cloud_msg, 0.0);
    }

    stringstream ss;
    ss << "Depth image received: w " << depth_msg->width << " h "
       << depth_msg->height << " frame_id " << depth_msg->header.frame_id
       << " camera_info "  << m_camera_info_msg.header.frame_id  <<"\n";

    ss << "Cloud message: " << cloud_msg->row_step* cloud_msg->height << "\n";
    ROS_INFO(ss.str().c_str());
    // setting frame id to transform image
    cloud_msg->header.frame_id = m_camera_info_msg.header.frame_id;

    sensor_msgs::PointCloud2 out;
    pcl_ros::transformPointCloud("base_footprint", *cloud_msg, out,
                                 *m_tf_listener);
    m_viewer->removePointCloud("view");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    m_viewer->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "view");
    stringstream ss1;
    ss1 << "Size of image data: " << sizeof(cloud_msg->data);
    ROS_INFO(ss1.str().c_str());





  }

 private:
  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::ServiceClient m_client;
  ros::Subscriber m_kinect_subscriber, m_camera_info_sub;
  ros::Timer m_timer;
  bool m_first_time;
  shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
  unique_ptr<tf::TransformListener> m_tf_listener;

  chrono::time_point<chrono::system_clock> m_kinect_start, m_kinect_end,
      m_laser_start, m_laser_end;
  chrono::time_point<chrono::system_clock> m_kinect_last_received;
  float m_kinect_timeout_ms;

  sensor_msgs::CameraInfo m_camera_info_msg;
  std::mutex m_camera_mutex;


};

}  // end namepace

int main(int argc, char** argv) {
  ROS_INFO("Init viewer!");
  shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = shared_ptr<pcl::visualization::PCLVisualizer>(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  ROS_INFO("Init listener!");
  ros::init(argc, argv, "composite_cloud_builder");
  // ros::NodeHandle nh;
  // LaserScannerListener scanner_listener(nh, viewer);

  amazon_challenge::PeriodicCloudPublisher laserPublisher(viewer);


  ROS_INFO("Looping!");
  while (!viewer->wasStopped()) {

    ros::spinOnce();
    viewer->spinOnce(1);
  }

  return 0;
}
