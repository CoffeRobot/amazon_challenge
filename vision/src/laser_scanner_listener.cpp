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
#include <memory>

// Services
#include "laser_assembler/AssembleScans2.h"
#include "laser_assembler/AssembleScans.h"

using namespace std;

class PeriodicLaserPublisher {

 public:
  PeriodicLaserPublisher(shared_ptr<pcl::visualization::PCLVisualizer> viewer) :
    m_nh(),
    m_viewer(viewer)
  {
    m_pub = m_nh.advertise<sensor_msgs::PointCloud>("assembled_cloud", 1);

    m_client = m_nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

    m_timer = m_nh.createTimer(ros::Duration(5, 0),
                               &PeriodicLaserPublisher::laserCallback, this);

    m_kinect_subscriber = m_nh.subscribe<sensor_msgs::PointCloud2>(
        "/head_mount_kinect/depth/points", 1, &PeriodicLaserPublisher::kinectCallback, this);

    m_tf_listener = unique_ptr<tf::TransformListener>(new tf::TransformListener());
  }

  void laserCallback(const ros::TimerEvent& e) {

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
      //ROS_INFO("Published Cloud with laser");
      pcl::fromROSMsg(msg, pcl_cloud);

      ROS_INFO("Published Cloud with %u points",
               (uint32_t)(pcl_cloud.points.size()));

      m_viewer->removePointCloud("laser_cloud");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
            pcl_cloud.makeShared(), 255, 0, 0);
      m_viewer->addPointCloud<pcl::PointXYZ>(pcl_cloud.makeShared(), single_color,
                                             "laser_cloud");

      //pub_.publish(srv.response.cloud);
    } else {
      ROS_ERROR("Error making service call\n");
    }
  }

  void kinectCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
      m_viewer->removePointCloud("kinect_cloud");
      sensor_msgs::PointCloud2 out;
      pcl_ros::transformPointCloud("base_link", *msg, out, *m_tf_listener);
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(out, cloud);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(
            cloud.makeShared(), 0, 0, 255);
      m_viewer->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), single_color,
                                             "kinect_cloud");
      ROS_INFO("Point cloud received!");
  }

 private:
  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::ServiceClient m_client;
  ros::Subscriber m_kinect_subscriber;
  ros::Timer m_timer;
  bool m_first_time;
  shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
  unique_ptr<tf::TransformListener> m_tf_listener;
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
  //ros::NodeHandle nh;
  //LaserScannerListener scanner_listener(nh, viewer);

  PeriodicLaserPublisher laserPublisher(viewer);

  ROS_INFO("Looping!");
  while (!viewer->wasStopped()) {

    ros::spinOnce();
    viewer->spinOnce(1);
  }

  return 0;
}
