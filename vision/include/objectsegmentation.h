#ifndef OBJECTSEGMENTATION_H
#define OBJECTSEGMENTATION_H

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <vector>

namespace amazon_challenge {

class ObjectSegmentation {
 public:
  ObjectSegmentation();

  void findPerpendicularPlane(const pcl::PointCloud<pcl::PointXYZ>& in_cloud,
                              pcl::PointCloud<pcl::PointXYZ>& plane_cloud,
                              pcl::PointCloud<pcl::PointXYZ>& filtered_cloud);

  void findPlane(const pcl::PointCloud<pcl::PointXYZ>& in_cloud,
                 pcl::PointCloud<pcl::PointXYZ>& plane_cloud,
                 pcl::PointCloud<pcl::PointXYZ>& filtered_cloud);

  bool filterCloudByHeight(const pcl::PointCloud<pcl::PointXYZ>& in,
                           const Eigen::Vector3f& bin_pos);

  void clusterComponentsEuclidean(
      pcl::PointCloud<pcl::PointXYZ>& in_cloud,
      std::vector<pcl::PointCloud<pcl::PointXYZ>>& clusters);

  void clusterExpectedComponents(
      int expected_clusters, pcl::PointCloud<pcl::PointXYZ>& in_cloud,
      const Eigen::Vector3f& bin_pos,
      std::vector<pcl::PointCloud<pcl::PointXYZ>>& clusters);

  void extractPose(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                   Eigen::Vector4f& centroid, Eigen::Quaternionf& rotation,
                   float& w, float& h, float& d);

  void findClusterBB(pcl::PointCloud<pcl::PointXYZ>& cloud_cluster,
                     Eigen::Vector4f& centroid, float& w, float& h, float& d);
};

}  // end namespace
#endif  // OBJECTSEGMENTATION_H
