#ifndef PERIODIC_CLOUD_UTILS
#define PERIODIC_CLOUD_UTILS

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointField.h>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl/filters/passthrough.h>
#include <DepthTraits.h>
#include <sstream>
#include <limits>

namespace amazon_challenge {

inline int addPointField(sensor_msgs::PointCloud2& cloud_msg,
                         const std::string& name, int count, int datatype,
                         int offset) {
  sensor_msgs::PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  cloud_msg.fields.push_back(point_field);

  // Update the offset
  return offset + point_field.count * 4;  // size of float 32
}

inline void setPointCloud2Fields(sensor_msgs::PointCloud2& cloud_msg,
                                 int num_fields) {
  cloud_msg.fields.clear();
  cloud_msg.fields.reserve(1);

  int offset = 0;
  offset = addPointField(cloud_msg, "x", 1, sensor_msgs::PointField::FLOAT32,
                         offset);
  offset = addPointField(cloud_msg, "y", 1, sensor_msgs::PointField::FLOAT32,
                         offset);
  offset = addPointField(cloud_msg, "z", 1, sensor_msgs::PointField::FLOAT32,
                         offset);
  offset += 4;

  // Resize the point cloud accordingly
  cloud_msg.point_step = offset;
  cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
  cloud_msg.data.resize(cloud_msg.height * cloud_msg.row_step);
}

// Handles float or uint16 depths
template <typename T>
void depthToCloud(const sensor_msgs::ImageConstPtr& depth_msg,
                  const image_geometry::PinholeCameraModel& model,
                  sensor_msgs::PointCloud2& cloud_msg, double range_max = 0.0) {

  // init cloud message
  cloud_msg.header = depth_msg->header;
  cloud_msg.height = depth_msg->height;
  cloud_msg.width = depth_msg->width;
  cloud_msg.is_dense = false;
  cloud_msg.is_bigendian = false;

  setPointCloud2Fields(cloud_msg, 1);

  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for
  // computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters(T(1));
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  float* cloud_data_ptr = reinterpret_cast<float*>(&cloud_msg.data[0]);
  int row_offset = cloud_msg.width * 4;
  for (int v = 0; v < (int)cloud_msg.height; ++v, depth_row += row_step) {
    for (int u = 0; u < (int)cloud_msg.width; ++u) {
      T depth = depth_row[u];
      int data_id = (u * 4) + (v * row_offset);

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth)) {
        if (range_max != 0.0) {
          depth = DepthTraits<T>::fromMeters(range_max);
        } else {
          depth = bad_point;
          continue;
        }
      }

      cloud_data_ptr[data_id] = (u - center_x) * depth * constant_x;
      cloud_data_ptr[data_id + 1] = (v - center_y) * depth * constant_y;
      cloud_data_ptr[data_id + 2] = DepthTraits<T>::toMeters(depth);
    }
  }
}

Eigen::Vector3f getBinSize(std::string bin_name) {
  if (bin_name.compare("shelf_bin_A") == 0 ||
      bin_name.compare("shelf_bin_C") == 0 ||
      bin_name.compare("shelf_bin_J") == 0 ||
      bin_name.compare("shelf_bin_L") == 0) {
    return Eigen::Vector3f(0.42, 0.27, 0.24);
  } else if (bin_name.compare("shelf_bin_D") == 0 ||
             bin_name.compare("shelf_bin_F") == 0 ||
             bin_name.compare("shelf_bin_G") == 0 ||
             bin_name.compare("shelf_bin_I") == 0) {
    return Eigen::Vector3f(0.42, 0.27, 0.22);
  } else if (bin_name.compare("shelf_bin_B") == 0 ||
             bin_name.compare("shelf_bin_K") == 0) {
    return Eigen::Vector3f(0.42, 0.30, 0.22);
  } else {
    return Eigen::Vector3f(0.42, 0.30, 0.24);
  }
}

Eigen::Vector3i getBinColor(std::string bin_name) {
  if (bin_name.compare("shelf_bin_A") == 0) return Eigen::Vector3i(255, 0, 0);
  if (bin_name.compare("shelf_bin_B") == 0) return Eigen::Vector3i(0, 255, 0);
  if (bin_name.compare("shelf_bin_C") == 0) return Eigen::Vector3i(0, 0, 255);
  if (bin_name.compare("shelf_bin_D") == 0) return Eigen::Vector3i(0, 255, 255);
  if (bin_name.compare("shelf_bin_E") == 0) return Eigen::Vector3i(255, 0, 255);
  if (bin_name.compare("shelf_bin_F") == 0) return Eigen::Vector3i(255, 255, 0);
  if (bin_name.compare("shelf_bin_G") == 0) return Eigen::Vector3i(255, 0, 0);
  if (bin_name.compare("shelf_bin_H") == 0) return Eigen::Vector3i(0, 255, 0);
  if (bin_name.compare("shelf_bin_I") == 0) return Eigen::Vector3i(0, 0, 255);
  if (bin_name.compare("shelf_bin_J") == 0) return Eigen::Vector3i(0, 255, 255);
  if (bin_name.compare("shelf_bin_K") == 0) return Eigen::Vector3i(255, 0, 255);
  if (bin_name.compare("shelf_bin_L") == 0)
    return Eigen::Vector3i(255, 255, 0);
  else
    return Eigen::Vector3i(255, 255, 255);
}

void colorCloudWithBin(std::string bin_name, tf::Vector3& origin,
                       pcl::PointCloud<pcl::PointXYZRGB>& out_cloud) {
  auto size = getBinSize(bin_name);
  auto color = getBinColor(bin_name);

  float min_x = origin.x();
  float max_x = min_x + size.x();

  float min_y = origin.y();
  float max_y = min_y + size.y();

  float min_z = origin.z();
  float max_z = min_z + size.z();

  for (auto pt : out_cloud.points) {
    if (pt.x >= min_x && pt.x < max_x && pt.y > min_y && pt.y < max_y &&
        pt.z >= min_z && pt.z < max_z) {
      pt.r = color.x();
      pt.g = color.y();
      pt.b = color.z();
    }
  }
}

void cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr,
               float min_x, float max_x, float min_y, float max_y, float min_z,
               float max_z, pcl::PointCloud<pcl::PointXYZ>::Ptr& cropped) {
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_ptr);

  pass.setFilterFieldName("x");
  pass.setFilterLimits(min_x, max_x);
  pass.filter(*cropped);

  pass.setInputCloud(cropped);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(min_y, max_y);
  pass.filter(*cropped);

  pass.setInputCloud(cropped);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_z, max_z);
  pass.filter(*cropped);
}

void cropCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_ptr,
               float min_x, float max_x, float min_y, float max_y, float min_z,
               float max_z, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cropped) {
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_ptr);

  std::stringstream ss;
  //ss << "x: " << cloud_ptr->points.size() << " " << min_x << " " << max_x;
  pass.setFilterFieldName("x");
  pass.setFilterLimits(min_x, max_x);
  pass.filter(*cropped);
  //ss << " y: " << cropped->points.size() << " " << min_y << " " << max_y;
  pass.setInputCloud(cropped);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(min_y, max_y);
  pass.filter(*cropped);
  //ss << " z: " << cropped->points.size() << " " << min_z << " " << max_z;
  pass.setInputCloud(cropped);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_z, max_z);
  pass.filter(*cropped);
  //ss << " f: " << cropped->points.size();
  //ROS_INFO(ss.str().c_str());
}

bool getTimedTransform(const tf::TransformListener& listener,
                       std::string target_frame, std::string dest_frame,
                       float sec, tf::StampedTransform& transform) {
  ros::Time begin = ros::Time::now();
  ros::Rate rate(5.0);
  while ((ros::Time::now() - begin).toSec() < sec) {
    try {
      listener.lookupTransform(target_frame, dest_frame, ros::Time(0),
                               transform);
      return true;
    }
    catch (tf::TransformException& ex) {
        rate.sleep();
    }
  }
  std::stringstream ss;
  ss << "TF: timeout for " << target_frame << " " << dest_frame;
  ROS_ERROR(ss.str().c_str());
  return false;
}

void findBestMatching(const std::vector<std::vector<double>>& in_values, std::vector<int>& labels)
{
    /*vector<vector<double>> values;
    vector<double> v1 = {1, 4, 3};
    vector<double> v2 = {4, 2, 3};
    vector<double> v3 = {5,1,2};
    values.push_back(v1);
    values.push_back(v2);
    values.push_back(v3);*/

    if (in_values.size() == 0)
        return;
    auto values = in_values;
    labels.resize(values[0].size(),-1);

    for (auto i = 0; i < values.size(); ++i) {

        auto min_id_j = 0;
        auto min_id_k = 0;
        auto min_val = std::numeric_limits<double>::max();

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

        for(auto w = 0; w < values[min_id_j].size(); ++w)
            values[min_id_j][w] = std::numeric_limits<double>::max();

        for(auto w = 0; w < values.size(); ++w)
            values[w][min_id_k] = std::numeric_limits<double>::max();

        labels[min_id_k] = min_id_j;
    }
}

}  // end namespace

#endif
