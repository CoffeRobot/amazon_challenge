#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "message_filters/subscriber.h"
#include <sensor_msgs/CameraInfo.h>
#include <memory>
#include <chrono>
#include <sstream>


using namespace std;

namespace amazon_challenge{

class CloudPublisher{

public:
    CloudPublisher():
        m_nh(),
        m_publish_rate(1.0f)
    {
        m_subscriber = m_nh.subscribe<sensor_msgs::Image>(
                    "/head_mount_kinect/depth_registered/image_rect_raw/compressedDepth",1,
                    &CloudPublisher::imageCallback, this
                    );
        //m_publisher = m_nh.advertise<sensor_msgs::PointCloud2>("amazon_pr2_cloud");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
    {

    }


private:
    ros::NodeHandle m_nh;
    ros::Publisher m_publisher;
    ros::Subscriber m_subscriber;
    chrono::time_point<chrono::system_clock> m_last_cloud_published;
    float m_publish_rate;
};




} // end namespace
