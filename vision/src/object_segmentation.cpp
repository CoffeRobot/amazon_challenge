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

using namespace std;

namespace amazon_challenge {

class CloudSegmenter {

 public:
  CloudSegmenter() : m_nh() {
    m_service_server = m_nh.advertiseService(
        "receive_point_cloud", &CloudSegmenter::receivePointCloud, this);
    m_segmentation_pub = m_nh.advertise<sensor_msgs::PointCloud2>("segmentation_cloud", 1);
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
    return true;
  }

  void publishSegmentation() {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(m_cloud, cloud);
    m_segmentation_pub.publish(cloud);
  }

 private:
  ros::NodeHandle m_nh;
  ros::Publisher m_segmentation_pub;
  ros::ServiceServer m_service_server;
  tf::TransformListener m_tf_listener;
  visualization_msgs::Marker m_shelf_marker;

  std::mutex m_mutex;
  pcl::PointCloud<pcl::PointXYZ> m_cloud;
};
}
int main(int argc, char **argv) {

  ros::init(argc, argv, "object_segmentation_node");

  amazon_challenge::CloudSegmenter cs;

  ros::Rate r(100);
  while (ros::ok()) {
    cs.publishSegmentation();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
