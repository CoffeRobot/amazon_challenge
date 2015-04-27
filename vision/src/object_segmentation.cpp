#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


using namespace std;

namespace amazon_challenge{


class CloudSegmenter{


public:

    CloudSegmenter():
        m_nh()
    {
        m_marker_publisher = m_nh.advertise<visualization_msgs::Marker>("segmentation_marker", 10);
    };


    void publishMarkers()
    {
        tf::StampedTransform transform;
        string target_frame = "base_footprint";
        string dest_frame = "shelf_frame";

        try{
            m_tf_listener.lookupTransform(target_frame, dest_frame, ros::Time(0), transform);

        }
        catch (tf::TransformException& ex) {
          ROS_ERROR("%s", ex.what());
          return;
        }

        m_shelf_marker.header.frame_id = target_frame;
        m_shelf_marker.header.stamp = ros::Time::now();
        m_shelf_marker.type = visualization_msgs::Marker::CUBE;
        m_shelf_marker.ns = "basic_shape";
        m_shelf_marker.id = 0;
        m_shelf_marker.action = visualization_msgs::Marker::ADD;

        auto origin = transform.getOrigin();

        m_shelf_marker.pose.position.x = origin.x() + 0.43;
        m_shelf_marker.pose.position.y = origin.y();
        m_shelf_marker.pose.position.z = origin.z() + 0.88;
        m_shelf_marker.pose.orientation.x = 0.0;
        m_shelf_marker.pose.orientation.y = 0.0;
        m_shelf_marker.pose.orientation.z = 0.0;
        m_shelf_marker.pose.orientation.w = 1.0;

        m_shelf_marker.scale.x = 1.0;
        m_shelf_marker.scale.y = 1.0;
        m_shelf_marker.scale.z = 2.4;

        m_shelf_marker.color.r = 0.0f;
        m_shelf_marker.color.g = 1.0f;
        m_shelf_marker.color.b = 0.0f;
        m_shelf_marker.color.a = 0.3;

        m_shelf_marker.lifetime = ros::Duration();

        m_marker_publisher.publish(m_shelf_marker);

    }

private:

    ros::NodeHandle m_nh;
    ros::Publisher m_marker_publisher;
    tf::TransformListener m_tf_listener;
    visualization_msgs::Marker m_shelf_marker;



};




}
int main(int argc, char** argv) {

  ros::init(argc, argv, "object_segmentation_node");

  amazon_challenge::CloudSegmenter cs;

  ros::Rate r(100);
  while(ros::ok())
  {
    cs.publishMarkers();
    ros::spinOnce();
    r.sleep();
  }



  return 0;
}
