#include "../include/objectsegmentation.h"

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

void ObjectSegmentation::clusterComponentsEuclidean(
    pcl::PointCloud<pcl::PointXYZ> &in_cloud,
    vector<pcl::PointCloud<pcl::PointXYZ>> &clusters) {
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(in_cloud.makeShared());

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.005);  // 2cm
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

void ObjectSegmentation::findClusterOBB(
    pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, Eigen::Vector3f &position,
    Eigen::Quaternionf &rotation, float &w, float &h, float &d) {
  /*for (auto i = 0; i < m_found_clusters.size(); ++i) {
    pcl::PCA<pcl::PointXYZ> pca;
    pcl::PointCloud<pcl::PointXYZ> proj;

    pca.setInputCloud(m_found_clusters[i].makeShared());
    pca.project(m_found_clusters[i], proj);

    pcl::PointXYZ proj_min, proj_max;
    pcl::getMinMax3D(proj, proj_min, proj_max);

    rotation = Eigen::Quaternionf(pca.getEigenVectors());

    Eigen::Vector4f t = pca.getMean();
    position = Eigen::Vector3f(t.x(), t.y(), t.z());

    float width = fabs(proj_max.x - proj_min.x);
    float height = fabs(proj_max.y - proj_min.y);
    float depth = fabs(proj_max.z - proj_min.z);
  }*/
}

void ObjectSegmentation::findClusterBB(
    pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, Eigen::Vector4f& centroid,
    float &w, float &h, float &d)
{
    Eigen::Vector4f min;
    Eigen::Vector4f max;
    pcl::compute3DCentroid(cloud_cluster, centroid);
    pcl::getMinMax3D(cloud_cluster, min, max);
}

}  // end namespace
