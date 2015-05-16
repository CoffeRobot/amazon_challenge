#ifndef OBJECTRECOGNITION_H
#define OBJECTRECOGNITION_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
#include <string>

namespace amazon_challenge {

class ObjectRecognition {
 public:
  ObjectRecognition();

  void classifyClusters(const cv::Mat& rgb,
                        const std::vector<std::string> items,
                        const std::vector<cv::Rect> clusters,
                        std::vector<std::vector<float>>& probs);

 private:
  void extractHistogram(const cv::Mat& src, const cv::Mat& mask,
                        std::vector<cv::Mat>& hists);

  void extractHSHistograms(const cv::Mat& src, const cv::Mat& mask,
  std::vector<cv::Mat>& hists);

  void loadData();

  float histogramInterserction(const std::vector<cv::Mat>& fst,
                               const std::vector<cv::Mat>& scd);

  void project_histograms(const cv::Mat &in, cv::Mat& out);

  std::map<std::string, std::vector<cv::Mat>> m_rgb_models;
  std::map<std::string, std::vector<cv::Mat>> m_hs_models;

};

}  // end namespace

#endif  // OBJECTRECOGNITION_H
