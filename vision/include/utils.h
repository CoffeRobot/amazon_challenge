#ifndef PERIODIC_CLOUD_UTILS
#define PERIODIC_CLOUD_UTILS

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointField.h>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>

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
  for (int v = 0; v < (int)cloud_msg.height; ++v, depth_row += row_step)
  {
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
      cloud_data_ptr[data_id+1] = (v - center_y) * depth * constant_y;
      cloud_data_ptr[data_id+2] = DepthTraits<T>::toMeters(depth);

    }
  }

}

}  // end namespace

#endif
