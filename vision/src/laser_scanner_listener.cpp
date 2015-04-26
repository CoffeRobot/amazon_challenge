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
#include <memory>
#include <chrono>
#include <sstream>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include "../include/DepthTraits.h"


// Services
#include "laser_assembler/AssembleScans2.h"

using namespace std;

namespace amazon_challenge{

class PeriodicCloudPublisher {

 public:
  PeriodicCloudPublisher(shared_ptr<pcl::visualization::PCLVisualizer> viewer)
      : m_nh(),
        m_viewer(viewer),
        m_kinect_timeout_ms(1000),
        m_kinect_last_received(chrono::system_clock::now()) {
    m_pub = m_nh.advertise<sensor_msgs::PointCloud>("assembled_cloud", 1);

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

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& rgb_info_msg) {
    stringstream ss;
    ss << "CAMERA: " << rgb_info_msg;
    m_camera_info_msg = *rgb_info_msg;
    ROS_INFO(ss.str().c_str());
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

  void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg)
  {
    ROS_INFO("Depth image received");

    m_viewer->removePointCloud("kinect_cloud");

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(m_camera_info_msg);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg = cloud.makeShared();

    depthToCloud<uint16_t>(depth_msg, msg, model, 10000.0);

    std::cout << "Size of cloud " << msg->points.size() << std::endl;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        single_color(msg, 0, 0, 255);
    m_viewer->addPointCloud<pcl::PointXYZ>(msg, single_color,
                                           "kinect_cloud");
  }

  void kinectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    auto elapsed = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now() - m_kinect_last_received).count();

    if (static_cast<float>(elapsed) < m_kinect_timeout_ms) return;

    m_kinect_last_received = chrono::system_clock::now();

    float total = 0;
    m_kinect_start = chrono::system_clock::now();
    m_viewer->removePointCloud("kinect_cloud");
    m_kinect_end = chrono::system_clock::now();
    auto ms = chrono::duration_cast<chrono::milliseconds>(
        m_kinect_end - m_kinect_start).count();
    stringstream ss;
    ss << "Kinect frame pofile:\n";
    ss << "VW Remove cloud: " << ms << " ms\n";
    total += ms;

    m_kinect_start = chrono::system_clock::now();
    sensor_msgs::PointCloud2 out;
    pcl_ros::transformPointCloud("base_link", *msg, out, *m_tf_listener);
    m_kinect_end = chrono::system_clock::now();
    ms = chrono::duration_cast<chrono::milliseconds>(m_kinect_end -
                                                     m_kinect_start).count();
    ss << "Transform cloud: " << ms << " ms\n";
    total += ms;

    m_kinect_start = chrono::system_clock::now();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(out, cloud);
    m_kinect_end = chrono::system_clock::now();
    ms = chrono::duration_cast<chrono::milliseconds>(m_kinect_end -
                                                     m_kinect_start).count();
    ss << "ROS2PCL: " << ms << " ms\n";
    total += ms;

    m_kinect_start = chrono::system_clock::now();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        single_color(cloud.makeShared(), 0, 0, 255);
    m_viewer->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), single_color,
                                           "kinect_cloud");
    m_kinect_end = chrono::system_clock::now();
    ms = chrono::duration_cast<chrono::milliseconds>(m_kinect_end -
                                                     m_kinect_start).count();
    ss << "VWAdd: " << ms << " ms\n";
    total += ms;

    ss << "Total: " << total << " ms\n";

    ROS_INFO(ss.str().c_str());

    ROS_INFO("Point cloud received!");
  }

 private:

  // Handles float or uint16 depths
  template<typename T>
  void depthToCloud(
      const sensor_msgs::ImageConstPtr& depth_msg,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg,
      const image_geometry::PinholeCameraModel& model,
      double range_max)
  {
    // Use correct principal point from calibration
    float center_x = model.cx();
    float center_y = model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = DepthTraits<T>::toMeters( T(1) );
    float constant_x = unit_scaling / model.fx();
    float constant_y = unit_scaling / model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud_msg->begin();
    const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(T);
    for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
    {
      for (int u = 0; u < (int)cloud_msg->width; ++u)
      {
        pcl::PointXYZ& pt = *pt_iter++;
        T depth = depth_row[u];

        // Missing points denoted by NaNs
        if (!DepthTraits<T>::valid(depth))
        {
          if (range_max != 0.0)
          {
            depth = DepthTraits<T>::fromMeters(range_max);
          }
          else
          {
            pt.x = pt.y = pt.z = bad_point;
            continue;
          }
        }

        // Fill in XYZ
        pt.x = (u - center_x) * depth * constant_x;
        pt.y = (v - center_y) * depth * constant_y;
        pt.z = DepthTraits<T>::toMeters(depth);
      }
    }
  }



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
};

class LaserScannerListener {

 public:
  ros::NodeHandle m_nh;
  laser_geometry::LaserProjection m_projector;
  tf::TransformListener m_listener;
  message_filters::Subscriber<sensor_msgs::LaserScan> m_laser_sub;
  tf::MessageFilter<sensor_msgs::LaserScan> m_laser_notifier;
  ros::Publisher m_scan_pub;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
  shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;

  LaserScannerListener(ros::NodeHandle nh,
                       shared_ptr<pcl::visualization::PCLVisualizer> viewer)
      : m_nh(nh),
        m_laser_sub(m_nh, "tilt_scan", 10),
        m_laser_notifier(m_laser_sub, m_listener, "base_link", 10),
        m_cloud((new pcl::PointCloud<pcl::PointXYZ>)),
        m_viewer(viewer) {
    m_laser_notifier.registerCallback(
        boost::bind(&LaserScannerListener::scanCallback, this, _1));
    m_laser_notifier.setTolerance(ros::Duration(0.01));
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    sensor_msgs::PointCloud2 cloud;
    try {
      m_projector.transformLaserScanToPointCloud("base_link", *scan_in, cloud,
                                                 m_listener);
      pcl::fromROSMsg(cloud, *m_cloud);
      ROS_INFO("Point cloud received!");
      m_viewer->removePointCloud("laser_scan");
      m_viewer->addPointCloud<pcl::PointXYZ>(m_cloud, "laser_scan");
    }
    catch (tf::TransformException& e) {
      std::cout << e.what();
      return;
    }

    // Do something with cloud.
  }
};

} // end namepace

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


