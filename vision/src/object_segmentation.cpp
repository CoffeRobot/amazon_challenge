#include "../include/objectsegmentation.h"
#include <limits>
#include <ros/ros.h>

using namespace std;

namespace amazon_challenge {

ObjectSegmentation::ObjectSegmentation() {}

void ObjectSegmentation::findPerpendicularPlane(
    const pcl::PointCloud<pcl::PointXYZ> &in_cloud,
    pcl::PointCloud<pcl::PointXYZ> &plane_cloud,
    pcl::PointCloud<pcl::PointXYZ> &filtered_cloud) {

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = in_cloud.makeShared();
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(0.1);

  seg.setInputCloud(cloud_ptr);

  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() > 0) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(plane_cloud);

    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(filtered_cloud);

  } else {
    filtered_cloud = in_cloud;
  }
}

void ObjectSegmentation::findPlane(
    const pcl::PointCloud<pcl::PointXYZ> &in_cloud,
    pcl::PointCloud<pcl::PointXYZ> &plane_cloud,
    pcl::PointCloud<pcl::PointXYZ> &filtered_cloud) {

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = in_cloud.makeShared();
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(0.1);

  seg.setInputCloud(cloud_ptr);

  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() > 0) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(plane_cloud);

    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(filtered_cloud);

  } else {
    filtered_cloud = in_cloud;
  }
}

bool ObjectSegmentation::filterCloudByHeight(
    const pcl::PointCloud<pcl::PointXYZ> &in_cloud,
    const Eigen::Vector3f &bin_pos) {

  Eigen::Vector4f centroid;
  Eigen::Quaternionf rotation;
  float w, h, d;
  extractPose(in_cloud, centroid, rotation, w, h, d);

  float min_z = centroid.z() - h / 2.0f;

  return min_z > bin_pos.z() + 0.14f;
}

void ObjectSegmentation::clusterComponentsEuclidean(
    pcl::PointCloud<pcl::PointXYZ> &in_cloud,
    vector<pcl::PointCloud<pcl::PointXYZ>> &clusters) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(in_cloud.makeShared());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.01);  // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(in_cloud.makeShared());
  ec.extract(cluster_indices);

  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

    pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
    tmp_cloud.header = in_cloud.header;

    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

      auto pt = in_cloud.points[*pit];
      pcl::PointXYZ p(pt.x, pt.y, pt.z);

      tmp_cloud.push_back(p);
    }

    clusters.push_back(tmp_cloud);
  }
}

void ObjectSegmentation::clusterExpectedComponents(
    int expected_clusters, pcl::PointCloud<pcl::PointXYZ> &in_cloud,
    const Eigen::Vector3f &bin_pos,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> &clusters) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(in_cloud.makeShared());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);

  std::vector<float> distances = {0.005, 0.01, 0.015, 0.02,
                                  0.025, 0.03, 0.035, 0.04};
  int min_num_clusters = numeric_limits<int>::max();

  stringstream ss;
  for (auto dis : distances) {
    // 2cm
    cluster_indices.clear();
    ec.setSearchMethod(tree);
    ec.setInputCloud(in_cloud.makeShared());
    ec.extract(cluster_indices);
    ec.setClusterTolerance(dis);

    if (cluster_indices.size() <= min_num_clusters &&
        cluster_indices.size() >= expected_clusters) {

      clusters.clear();
      for (auto it = cluster_indices.begin(); it != cluster_indices.end();
           ++it) {

        pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
        tmp_cloud.header = in_cloud.header;

        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

          auto pt = in_cloud.points[*pit];
          pcl::PointXYZ p(pt.x, pt.y, pt.z);

          tmp_cloud.push_back(p);
        }

        //NEW: add the finter inside the clustering procedure
        if (!filterCloudByHeight(tmp_cloud, bin_pos))
          clusters.push_back(tmp_cloud);
      }
      min_num_clusters = cluster_indices.size();
    }
    ss << "SEG: " << dis << " clusters " << clusters.size() << "\n";
  }
  ROS_INFO(ss.str().c_str());
}

void ObjectSegmentation::extractPose(
    const pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Vector4f &centroid,
    Eigen::Quaternionf &rotation, float &w, float &h, float &d) {
  pcl::compute3DCentroid(cloud, centroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(cloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(
      covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
  eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

  // move the points to the that reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block<3, 3>(0, 0) = eigDx.transpose();
  p2w.block<3, 1>(0, 3) = -1.f * (p2w.block<3, 3>(0, 0) * centroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ> cPoints;
  pcl::transformPointCloud(cloud, cPoints, p2w);

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(cPoints, min_pt, max_pt);
  Eigen::Vector3f mean_diag =
      0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // final transform
  Eigen::Quaternionf qfinal(eigDx);

  Eigen::Vector3f tfinal = eigDx * mean_diag + centroid.head<3>();

  rotation = qfinal;
  w = max_pt.x - min_pt.x;
  h = max_pt.y - min_pt.y;
  d = max_pt.z - min_pt.z;
}

void ObjectSegmentation::findClusterBB(
    pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, Eigen::Vector4f &centroid,
    float &w, float &h, float &d) {
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  pcl::compute3DCentroid(cloud_cluster, centroid);
  pcl::getMinMax3D(cloud_cluster, min, max);
}

}  // end namespace
