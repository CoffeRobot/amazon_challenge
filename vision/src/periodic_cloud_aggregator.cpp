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

// Services
#include <laser_assembler/AssembleScans2.h>
#include <vision/ReceiveCloud.h>
#include <simtrack_nodes/UpdateValidationPointCloud.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <pr2_msgs/SetLaserTrajCmd.h>

using namespace std;

namespace amazon_challenge {

enum Status {
  RUNNING,
  SUCCESS,
  FAILURE
};

class PeriodicCloudPublisher {

 public:
  PeriodicCloudPublisher()
      : m_nh(),
        m_kinect_timeout_ms(4000.0f),
        m_min_aggregated_cloud_time(10.0),
        m_kinect_last_received(chrono::system_clock::now()),
        m_first_dimage_received(false),
        m_action_server(
            m_nh, "periodic_cloud_aggregator",
            boost::bind(&PeriodicCloudPublisher::executeCB, this, _1), false),
        m_action_name("periodic_cloud_aggregator"),
        m_build_aggregated_cloud(false),
        m_bin_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        m_shelf_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    m_publisher = m_nh.advertise<sensor_msgs::PointCloud2>("periodic_cloud", 1);
    m_shelf_publisher =
        m_nh.advertise<sensor_msgs::PointCloud2>("periodic_cloud_shelf", 1);
    m_bin_publisher =
        m_nh.advertise<sensor_msgs::PointCloud2>("periodic_cloud_bin", 1);

    m_client =
        m_nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
    m_timer = m_nh.createTimer(ros::Duration(5.0f),
                               &PeriodicCloudPublisher::laserCallback, this);
    m_taskmanager_sub =
        m_nh.subscribe("/amazon_next_task", 10,
                       &PeriodicCloudPublisher::taskmanagerCallback, this);

    m_kinect_subscriber = m_nh.subscribe<sensor_msgs::Image>(
        "/head_mount_kinect/depth/image_raw", 1,
        &PeriodicCloudPublisher::depthCallback, this);
    m_camera_info_sub = m_nh.subscribe<sensor_msgs::CameraInfo>(
        "/head_mount_kinect/depth/camera_info", 1,
        &PeriodicCloudPublisher::cameraInfoCallback, this);

    m_tf_listener =
        unique_ptr<tf::TransformListener>(new tf::TransformListener());

    m_shelf_marker_pub =
        m_nh.advertise<visualization_msgs::Marker>("shelf_marker", 10);
    m_bin_marker_pub =
        m_nh.advertise<visualization_msgs::Marker>("bin_marker", 10);

    m_segmentation_client =
        m_nh.serviceClient<vision::ReceiveCloud>("receive_point_cloud");
    m_tracking_client =
        m_nh.serviceClient<simtrack_nodes::UpdateValidationPointCloud>(
            "simtrack/update_validation_point_cloud");
    m_pr2_laser_client = m_nh.serviceClient<pr2_msgs::SetLaserTrajCmd>(
        "laser_tilt_controller/set_traj_cmd");
    // init action server
    m_action_server.start();

    stopTiltScanner();
  }
  // returns the status to the client (Behavior Tree)
  void setStatus(int status) {
    // Set The feedback and result of BT.action
    m_feedback.status = status;
    m_result.status = m_feedback.status;
    // publish the feedback
    m_action_server.publishFeedback(m_feedback);
    // setSucceeded means that it has finished the action (it has returned
    // SUCCESS or FAILURE).
    m_action_server.setSucceeded(m_result);

    switch (status) {  // Print for convenience
      case SUCCESS:
        ROS_INFO("Action %s Succeeded", ros::this_node::getName().c_str());
        break;
      case FAILURE:
        ROS_INFO("Action %s Failed", ros::this_node::getName().c_str());
        break;
      default:
        break;
    }
  }

  void executeCB(const vision::BTGoalConstPtr& goal) {

    // publish info to the console for the user
    ROS_INFO("Starting Action");
    m_build_aggregated_cloud = true;
    m_laser_request_start = ros::Time::now();
    startTiltScanner();

    // start executing the action
    while (!m_aggregated_cloud_ready) {
      // check that preempt has not been requested by the client
      if (m_action_server.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("Action Halted");
        // set the action state to preempted
        m_action_server.setPreempted();
        // success = false;
        break;
      }
    }

    auto elapsed = (ros::Time::now() - m_laser_request_start).toSec();

    stringstream ss;
    ss << "CB: Cloud built in: " << elapsed << " secs";

    // building the clouds
    ROS_INFO(ss.str().c_str());
    sensor_msgs::PointCloud2 out_msg, shelf_msg, bin_msg;
    pcl::toROSMsg(m_aggregated_cloud, out_msg);
    pcl::toROSMsg(*m_bin_cloud, bin_msg);
    pcl::toROSMsg(*m_shelf_cloud, shelf_msg);

    // notify segmentation and tracking
    notifySegmentation(bin_msg);
    notifyTracker(out_msg);
    m_aggregated_cloud_ready = false;
    m_build_aggregated_cloud = false;
    stopTiltScanner();
    // communicate bt the success
    setStatus(SUCCESS);
  }

  void notifySegmentation(sensor_msgs::PointCloud2& cloud) {
    vision::ReceiveCloud srv;
    srv.request.cloud = cloud;
    // Make the service call
    if (m_segmentation_client.call(srv)) {
      bool result = srv.response.result;
      if (result) ROS_INFO("Segmentation node notified");
    } else {
      ROS_ERROR("Segmentation node not listening");
    }
  }

  void notifyTracker(sensor_msgs::PointCloud2& cloud) {
    simtrack_nodes::UpdateValidationPointCloud srv;
    srv.request.point_cloud = cloud;
    if (m_tracking_client.call(srv)) {
      bool result = srv.response.success;
      if (result) ROS_INFO("Tracking node notified");
    } else {
      ROS_ERROR("Tracking node not listening");
    }
  }

  bool getTransformOrigin(string dest_frame, tf::Vector3& origin) {

    tf::StampedTransform transform;
    string target_frame = "base_footprint";

    try {
      m_tf_listener->lookupTransform(target_frame, dest_frame, ros::Time(0),
                                     transform);
      origin = transform.getOrigin();
      return true;
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("Error in getting the transform of type %s", ex.what());
      return false;
    }
  }

  void stopTiltScanner() {
    vector<double> positions;
    positions.push_back(0);
    positions.push_back(0);
    //= {(d_top / d_top_h), (d_bottom / d_bottom_h)};
    auto durations = {ros::Duration(0), ros::Duration(10)};

    pr2_msgs::SetLaserTrajCmd srv;
    // srv.request.command.header.stamp = ros::Time::now();
    srv.request.command.profile = "linear";
    srv.request.command.position = positions;
    srv.request.command.time_from_start = durations;
    srv.request.command.max_velocity = 0;
    srv.request.command.max_acceleration = 0;

    if (!m_pr2_laser_client.call(srv)) {
      ROS_ERROR("PCA: cannot stop pr2 tilt");
    }
  }

  void startTiltScanner() {

    tf::Vector3 scanner_origin, bin_origin;
    if (!getTransformOrigin("laser_tilt_link", scanner_origin)) {
      return;
    }
    if (!getTransformOrigin("shelf_" + m_bin_name, bin_origin)) {
      return;
    }

    float z_shelf_laser = scanner_origin.z();

    float bin_offset = 0.05;
    float bin_size = 0.27;

    float bin_low_x = bin_origin.x();
    float bin_low_z = bin_origin.z() - bin_offset;

    float bin_up_x = bin_low_x;
    float bin_up_z = bin_low_z + bin_size + bin_offset + 0.05;

    float d_top = (bin_up_z - z_shelf_laser) * -1;
    float d_bottom = (bin_low_z - z_shelf_laser) * -1;

    float d_top_h = sqrt(pow((bin_up_z - scanner_origin.z()), 2) +
                         pow((bin_up_x - scanner_origin.x()), 2));
    float d_bottom_h = sqrt(pow((bin_low_z - scanner_origin.z()), 2) +
                            pow((bin_low_x - scanner_origin.x()), 2));

    double start = asin(d_top / d_top_h);
    double stop = asin(d_bottom / d_bottom_h);

    vector<double> positions;
    positions.push_back(start);
    positions.push_back(stop);
    //= {(d_top / d_top_h), (d_bottom / d_bottom_h)};
    auto durations = {ros::Duration(0), ros::Duration(10)};

    stringstream ss;
    ss << "TRAJ: " << positions[0] << " " << positions[1];
    ROS_INFO(ss.str().c_str());

    pr2_msgs::SetLaserTrajCmd srv;
    // srv.request.command.header.stamp = ros::Time::now();
    srv.request.command.profile = "linear";
    srv.request.command.position = positions;
    srv.request.command.time_from_start = durations;
    srv.request.command.max_velocity = 10;
    srv.request.command.max_acceleration = 30;

    if (!m_pr2_laser_client.call(srv)) {
      ROS_ERROR("PCA: cannot start pr2 tilt");
    }
  }

  void buildPointCloudMessage() {

    if (m_camera_info_msg.width == 0) return;
    if (m_last_depth_msg->width == 0) return;

    m_mutex.lock();
    // ROS_INFO("Mutex locked by Cloud message");
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(m_camera_info_msg);

    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);

    if (m_last_depth_msg->encoding ==
        sensor_msgs::image_encodings::TYPE_16UC1) {
      depthToCloud<uint16_t>(m_last_depth_msg, model, *cloud_msg, 0.0);
    } else if (m_last_depth_msg->encoding ==
               sensor_msgs::image_encodings::TYPE_32FC1) {
      depthToCloud<float>(m_last_depth_msg, model, *cloud_msg, 0.0);
    } else {
      ROS_WARN("Warning: unsupported depth image format!");
      m_mutex.unlock();
      // ROS_INFO("Mutex unlocked by cloud message");
      return;
    }

    // cloud_msg->header.frame_id = m_camera_info_msg.header.frame_id;
    cloud_msg->header.frame_id = m_last_depth_msg->header.frame_id;
    string s = "img2cloud " + cloud_msg->header.frame_id;
    ROS_INFO(s.c_str());
    m_mutex.unlock();

    sensor_msgs::PointCloud2 out;
    pcl_ros::transformPointCloud("base_footprint", *cloud_msg, out,
                                 *m_tf_listener);

    pcl::fromROSMsg(out, m_kinect_cloud);
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& rgb_info_msg) {
    stringstream ss;

    auto can_lock = m_mutex.try_lock();

    if (!can_lock)
      return;
    else {
      // ROS_INFO("Mutex locked by camera");
      m_camera_info_msg = *rgb_info_msg;
      m_mutex.unlock();
      // ROS_INFO("Mutex unlocked by camera");
    }
  }

  void labelCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    tf::Vector3 bin_origin;
    if (getTransformOrigin("shelf_bin_A", bin_origin)) {
      colorCloudWithBin("bin_A", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_B", bin_origin)) {
      colorCloudWithBin("bin_B", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_C", bin_origin)) {
      colorCloudWithBin("bin_C", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_D", bin_origin)) {
      colorCloudWithBin("bin_D", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_E", bin_origin)) {
      colorCloudWithBin("bin_E", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_F", bin_origin)) {
      colorCloudWithBin("bin_F", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_G", bin_origin)) {
      colorCloudWithBin("bin_G", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_H", bin_origin)) {
      colorCloudWithBin("bin_H", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_I", bin_origin)) {
      colorCloudWithBin("bin_I", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_J", bin_origin)) {
      colorCloudWithBin("bin_A", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_K", bin_origin)) {
      colorCloudWithBin("bin_A", bin_origin, cloud);
    }
    if (getTransformOrigin("shelf_bin_L", bin_origin)) {
      colorCloudWithBin("bin_L", bin_origin, cloud);
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

    if (!m_build_aggregated_cloud) {
      // ROS_INFO("Laser: Aggregated cloud not requested!");
      return;
    }

    auto secs = (ros::Time::now() - m_laser_request_start).toSec();

    if (secs < m_min_aggregated_cloud_time) return;

    // Populate our service request based on our timer callback times
    laser_assembler::AssembleScans2 srv;

    srv.request.begin = m_laser_request_start;
    srv.request.end = m_laser_request_start + ros::Duration(10);

    // Make the service call
    if (m_client.call(srv)) {

      auto elapsed = (ros::Time::now() - m_laser_request_start).toSec();

      sensor_msgs::PointCloud2 msg = srv.response.cloud;

      stringstream ss;
      ss << "LC: service callded after secs: " << elapsed;
      ROS_INFO(ss.str().c_str());

      if (!m_first_dimage_received) {
        // ROS_INFO("Kinect depth not available yet");
        return;
      }
      pcl::PointCloud<pcl::PointXYZ> laser_cloud;
      pcl::fromROSMsg(msg, laser_cloud);

      buildPointCloudMessage();

      m_aggregated_cloud = laser_cloud;
      m_aggregated_cloud += m_kinect_cloud;

      createCloudBin(m_aggregated_cloud);
      CreateCloudShelf(m_aggregated_cloud);

      m_aggregated_cloud_ready = true;

      elapsed = (ros::Time::now() - m_laser_request_start).toSec();

      stringstream ss1;
      ss1 << "Aggregated cloud built with points: "
          << m_aggregated_cloud.points.size() << " in " << elapsed << " secs";
      ROS_INFO(ss1.str().c_str());

    } else {
      ROS_ERROR("Error making service call\n");
    }
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {

    if (!m_build_aggregated_cloud) {
      // ROS_INFO("Kinect: Aggregated cloud not requested!");
      return;
    }

    auto elapsed = chrono::duration_cast<chrono::milliseconds>(
        chrono::system_clock::now() - m_kinect_last_received).count();

    if (static_cast<float>(elapsed) < m_kinect_timeout_ms) return;
    m_kinect_last_received = chrono::system_clock::now();

    ROS_INFO("Kinect depth received");

    auto can_lock = m_mutex.try_lock();

    if (can_lock) {
      // ROS_INFO("Mutex locked by kinect");
      m_last_depth_msg = depth_msg;
      if (!m_first_dimage_received) m_first_dimage_received = true;
      m_mutex.unlock();
      // ROS_INFO("Mutex unlocked by kinect");
    } else
      return;
  }

  void taskmanagerCallback(const std_msgs::String::ConstPtr& msg) {
    m_bin_name = "";
    m_obj_name = "";

    m_mutex.lock();

    for (auto i = 1; i < msg->data.length() - 1; i++) {
      if (i < 6)
        m_bin_name += msg->data[i];
      else if (i > 6)
        m_obj_name += msg->data[i];
    }

    m_mutex.unlock();
  }

  /*void publishShelfMarkers()
  {
      string target_frame = "base_footprint";
      string dest_frame = "shelf_frame";
      tf::StampedTransform transform;
      try {
        m_tf_listener->lookupTransform(target_frame, dest_frame, ros::Time(0),
                                       transform);
      }
      catch (tf::TransformException& ex) {
        ROS_ERROR("Error in shelf publish of type %s", ex.what());
        return;
      }
      auto origin = transform.getOrigin();
      visualization_msgs::Marker shelf_marker;
      getShelfMarker(shelf_marker);
      shelf_marker.header.frame_id = target_frame;
      shelf_marker.pose.position.x += origin.x();
      shelf_marker.pose.position.y += origin.y();
      shelf_marker.pose.position.z += origin.z();

      m_shelf_marker_pub.publish(shelf_marker);
  }*/

  void publishCurrentBinMarkers() {

    string target_frame = "base_footprint";
    string dest_frame = "shelf_" + m_bin_name;
    tf::StampedTransform transform;
    try {
      m_tf_listener->lookupTransform(target_frame, dest_frame, ros::Time(0),
                                     transform);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("Error in shelf publish of type %s", ex.what());
      return;
    }
    auto origin = transform.getOrigin();
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame;

    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.ns = "bin";
    marker.id = 1;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3;

    marker.lifetime = ros::Duration();

    if (m_bin_name.compare("bin_A") == 0 || m_bin_name.compare("bin_B") == 0 ||
        m_bin_name.compare("bin_C") == 0 || m_bin_name.compare("bin_J") == 0 ||
        m_bin_name.compare("bin_K") == 0 || m_bin_name.compare("bin_L") == 0) {
      marker.pose.position.x = origin.x() - 0.01;
      marker.pose.position.y = origin.y() - 0.03;
      marker.pose.position.z = origin.z() - 0.04;
      marker.scale.x = 0.44;
      marker.scale.y = 0.31;
      marker.scale.z = 0.32;
      // ROS_INFO("Big bin");

    } else if (m_bin_name.compare("bin_D") == 0 ||
               m_bin_name.compare("bin_E") == 0 ||
               m_bin_name.compare("bin_F") == 0 ||
               m_bin_name.compare("bin_G") == 0 ||
               m_bin_name.compare("bin_H") == 0 ||
               m_bin_name.compare("bin_I") == 0) {
      marker.pose.position.x = origin.x() + 0.21;
      marker.pose.position.y = origin.y() + 0.14;
      marker.pose.position.z = origin.z() + 0.11;
      marker.scale.x = 0.42;
      marker.scale.y = 0.28;
      marker.scale.z = 0.22;
      // ROS_INFO("Small bin");
    }
    m_bin_marker_pub.publish(marker);
  }

  void publishClouds() {

    sensor_msgs::PointCloud2 out_msg, bin_msg, shelf_msg;
    pcl::toROSMsg(m_colored_cloud, out_msg);
    pcl::toROSMsg(*m_bin_cloud, bin_msg);
    pcl::toROSMsg(*m_shelf_cloud, shelf_msg);

    // publish clouds for debugging purposes
    m_publisher.publish(out_msg);
    m_shelf_publisher.publish(shelf_msg);
    m_bin_publisher.publish(bin_msg);
  }

  void publishMarkers() { publishCurrentBinMarkers(); }

 private:
  bool getBinType(string bin_name) {
    if (m_bin_name.compare("bin_A") == 0 || m_bin_name.compare("bin_B") == 0 ||
        m_bin_name.compare("bin_C") == 0 || m_bin_name.compare("bin_J") == 0 ||
        m_bin_name.compare("bin_K") == 0 || m_bin_name.compare("bin_L") == 0) {
      return 0;

    } else {
      return 1;
    }
  }

  void CreateCloudShelf(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    tf::StampedTransform transform;
    string target_frame = "base_footprint";
    string dest_frame = "shelf_frame";

    try {
      m_tf_listener->lookupTransform(target_frame, dest_frame, ros::Time(0),
                                     transform);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("Error in shelf publish of type %s", ex.what());
      return;
    }
    auto origin = transform.getOrigin();

    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(cloud.makeShared());

    auto min_x = origin.x();
    auto max_x = origin.x() + 0.50;
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_x, max_x);
    pass.filter(*m_shelf_cloud);

    auto min_y = origin.y() - 0.50;
    auto max_y = origin.y() + 0.50;
    pass.setInputCloud(m_shelf_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*m_shelf_cloud);

    auto min_z = origin.z();
    auto max_z = origin.z() + 2.0;
    pass.setInputCloud(m_shelf_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*m_shelf_cloud);
  }

  void createCloudBin(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    tf::StampedTransform transform;
    string target_frame = "base_footprint";
    m_mutex.lock();
    string dest_frame = "shelf_" + m_bin_name;
    m_mutex.unlock();

    try {
      m_tf_listener->lookupTransform(target_frame, dest_frame, ros::Time(0),
                                     transform);
    }
    catch (tf::TransformException& ex) {
      ROS_ERROR("Erro in bin publish of type %s", ex.what());
      return;
    }
    auto origin = transform.getOrigin();

    float min_x, max_x, min_y, max_y, min_z, max_z;

    auto size = getBinSize(m_bin_name);
    float filter_offset = 0.02;
    min_x = 0.04;
    min_y = 0.02;
    min_z = 0.02;
    max_x = size.x() - filter_offset;
    max_y = size.y() - filter_offset;
    max_z = size.z() - filter_offset;

    // cleaning the old cloud
    m_bin_cloud->clear();

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud.makeShared());

    auto min_range = origin.x() + min_x;
    auto max_range = origin.x() + max_x;
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min_range, max_range);
    pass.filter(*m_bin_cloud);

    min_range = origin.y() + min_y;
    max_range = origin.y() + max_y;
    pass.setInputCloud(m_bin_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_range, max_range);
    pass.filter(*m_bin_cloud);

    float min_range_z = origin.z() + min_z;
    float max_range_z = origin.z() + max_z;
    pass.setInputCloud(m_bin_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_range_z, max_range_z);
    pass.filter(*m_bin_cloud);
  }

  // publishers
  ros::Publisher m_publisher, m_shelf_publisher, m_bin_publisher;
  // service clients
  ros::ServiceClient m_client, m_segmentation_client, m_tracking_client,
      m_pr2_laser_client;
  // subscribers
  ros::Subscriber m_kinect_subscriber, m_camera_info_sub, m_taskmanager_sub;
  // timers
  ros::Timer m_timer;
  bool m_first_time;
  bool m_first_dimage_received;
  chrono::time_point<chrono::system_clock> m_kinect_start, m_kinect_end,
      m_laser_start, m_laser_end;
  chrono::time_point<chrono::system_clock> m_kinect_last_received;
  float m_kinect_timeout_ms;
  ros::Time m_laser_request_start;
  float m_min_aggregated_cloud_time;

  unique_ptr<tf::TransformListener> m_tf_listener;

  string m_bin_name;
  string m_obj_name;

  ros::ServiceServer m_bin_cloud_server;

  pcl::PointCloud<pcl::PointXYZ> m_aggregated_cloud, m_kinect_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_bin_cloud, m_shelf_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> m_colored_cloud;

  sensor_msgs::CameraInfo m_camera_info_msg;
  sensor_msgs::ImageConstPtr m_last_depth_msg;
  std::mutex m_mutex;

  std::atomic_bool m_build_aggregated_cloud, m_aggregated_cloud_ready;

  ros::Publisher m_shelf_marker_pub, m_bin_marker_pub;

 protected:
  ros::NodeHandle m_nh;
  actionlib::SimpleActionServer<vision::BTAction> m_action_server;
  std::string m_action_name;
  vision::BTFeedback m_feedback;
  vision::BTResult m_result;
};

}  // end namepace

int main(int argc, char** argv) {
  ROS_INFO("Init viewer!");

  ROS_INFO("Init listener!");
  ros::init(argc, argv, "periodic_cloud_aggregator");
  // ros::NodeHandle nh;
  // LaserScannerListener scanner_listener(nh, viewer);

  amazon_challenge::PeriodicCloudPublisher aggregated_cloud_publisher;

  ros::Rate r(100);
  while (ros::ok()) {
    aggregated_cloud_publisher.publishClouds();
    aggregated_cloud_publisher.publishMarkers();
    // aggregated_cloud_publisher.publishShelfMarkers();
    // aggregated_cloud_publisher.publishCurrentBinMarkers();
    ros::spinOnce();

    r.sleep();
  }

  return 0;
}
