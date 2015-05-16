#include "../include/objectrecognition.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

namespace amazon_challenge {

ObjectRecognition::ObjectRecognition() { loadData(); }

void ObjectRecognition::classifyClusters(
    const cv::Mat &rgb, const vector<string> items,
    const vector<cv::Rect> clusters, std::vector<std::vector<float>> &probs) {
  probs.resize(items.size(), vector<float>(clusters.size(), 0));

  for (auto i = 0; i < items.size(); ++i) {
    auto model = m_rgb_models.find(items[i]);
    stringstream ss;
    for (auto j = 0; j < clusters.size(); ++j) {

      ss << "hist " << i << " " << j << " " << items[i];

      if (model != m_rgb_models.end()) {
        Mat1b mask(rgb.rows, rgb.cols, static_cast<uchar>(0));
        rectangle(mask, clusters[j], 255, -1);
        vector<Mat> histogram(3, Mat());
        extractHistogram(rgb, mask, histogram);
        float prob = histogramInterserction(histogram, model->second);
        probs[i][j] = prob;
        ss << " " << prob;
        ss << " " << clusters[j];

      } else {
        probs[i][j] = 0;
      }
    }

    // ROS_INFO(ss.str().c_str());
  }
}

void ObjectRecognition::extractHistogram(const Mat &src, const Mat &mask,
                                         std::vector<cv::Mat> &hists) {

  vector<Mat> bgr_planes;
  split(src, bgr_planes);

  /// Establish the number of bins
  int histSize = 30;

  /// Set the ranges ( for B,G,R) )
  float range[] = {0, 256};
  const float *histRange = {range};

  bool uniform = true;
  bool accumulate = false;

  /// Compute the histograms:
  calcHist(&bgr_planes[0], 1, 0, mask, hists[0], 1, &histSize, &histRange,
           uniform, accumulate);
  calcHist(&bgr_planes[1], 1, 0, mask, hists[1], 1, &histSize, &histRange,
           uniform, accumulate);
  calcHist(&bgr_planes[2], 1, 0, mask, hists[2], 1, &histSize, &histRange,
           uniform, accumulate);

  std::cout << "here 2";

  float num_pixels;
  if (!mask.empty())
    num_pixels = cv::countNonZero(mask);
  else
    num_pixels = src.cols * src.rows;
  for (auto i = 0; i < histSize; ++i) {
    hists[0].at<float>(0, i) = hists[0].at<float>(0, i) / num_pixels;
    hists[1].at<float>(0, i) = hists[1].at<float>(0, i) / num_pixels;
    hists[2].at<float>(0, i) = hists[2].at<float>(0, i) / num_pixels;
  }
}

void ObjectRecognition::extractHSHistograms(const Mat &src, const Mat &mask,
                                            std::vector<Mat> &hists)
{
    float h_range[] = { 0, 179 };
    const float *hRange = {h_range};
    float s_range[] = { 0, 255 };
    const float *sRange = {s_range};
    int h_bins = 30; int s_bins = 32;

    Mat hsv;
    cvtColor( src, hsv, COLOR_BGR2HSV );
    vector<Mat> hsv_planes;
    split(hsv, hsv_planes);

    bool uniform = true;
    bool accumulate = false;

    /// Compute the histograms:
    calcHist(&hsv_planes[0], 1, 0, mask, hists[0], 1, &h_bins, &hRange,
             uniform, accumulate);
    calcHist(&hsv_planes[1], 1, 0, mask, hists[1], 1, &s_bins, &sRange,
             uniform, accumulate);

    float num_pixels;
    if (!mask.empty())
      num_pixels = cv::countNonZero(mask);
    else
      num_pixels = src.cols * src.rows;

    for (auto i = 0; i < h_bins; ++i) {
      hists[0].at<float>(0, i) = hists[0].at<float>(0, i) / num_pixels;
    }

    for (auto i = 0; i < s_bins; ++i) {
      hists[1].at<float>(0, i) = hists[1].at<float>(0, i) / num_pixels;
    }

}


void ObjectRecognition::loadData() {
  Mat rgb = imread(
      "/home/amazon/amazon_challenge_ws/src/amazon_challenge/data/"
      "histogram_models/kyjen_squeakin_eggs_plush_puppies.png");

  vector<Mat> histogram(3, Mat());
  extractHistogram(rgb, Mat(), histogram);
  m_rgb_models.insert(pair<string, vector<Mat>>(
      "kyjen_squeakin_eggs_plush_puppies", histogram));
  vector<Mat> hsv_0(2, Mat());
  extractHSHistograms(rgb, Mat(), hsv_0);
  m_hs_models.insert(pair<string, vector<Mat>>(
                         "kyjen_squeakin_eggs_plush_puppies", hsv_0));

  rgb = imread(
      "/home/amazon/amazon_challenge_ws/src/amazon_challenge/data/"
      "histogram_models/first_years_take_and_toss_straw_cup.png");
  vector<Mat> histogram2(3, Mat());
  extractHistogram(rgb, Mat(), histogram2);
  m_rgb_models.insert(pair<string, vector<Mat>>(
      "first_years_take_and_toss_straw_cup", histogram2));
  vector<Mat> hsv_1(2, Mat());
  extractHSHistograms(rgb, Mat(), hsv_1);
  m_hs_models.insert(pair<string, vector<Mat>>(
                         "first_years_take_and_toss_straw_cup", hsv_1));

  stringstream ss;
  ss << "DIFFERENCE: " << histogramInterserction(histogram, histogram2);
  ROS_INFO(ss.str().c_str());
  /*
  ofstream file(
      "/home/amazon/amazon_challenge_ws/src/amazon_challenge/data/hist.txt",
      std::ofstream::out);
  stringstream ss;

  for (auto i = 0; i < histogram.size(); ++i) {
    Mat &fst_c = histogram[i];
    for (auto j = 0; j < fst_c.rows; ++j) {
      ss << fst_c.at<float>(0, j) << ",";
    }

    ss << "\n";
  }

  float val = histogramInterserction(histogram, histogram);

  ss << "intersection " << val;

  file << ss.str();
  file.close();*/
}

float ObjectRecognition::histogramInterserction(const std::vector<Mat> &fst,
                                                const std::vector<Mat> &scd) {
  float result = 0;

  for (auto i = 0; i < fst.size(); ++i) {
    const Mat &fst_c = fst[i];
    const Mat &scd_c = scd[i];

    for (auto j = 0; j < fst_c.rows; ++j) {
      result += std::min(fst_c.at<float>(0, j), scd_c.at<float>(0, j));
    }
  }
  return result / 3.0f;
}

void ObjectRecognition::project_histograms(const Mat&in, Mat &out)
{
    cv::Mat hsv;
    cvtColor( in, hsv, COLOR_BGR2HSV );

    auto model = m_hs_models.find("first_years_take_and_toss_straw_cup");
    float h_range[] = { 0, 179 };
     const float* ranges = { h_range };

    calcBackProject( &hsv, 1, 0, MatND(model->second[0]), out, &ranges, 1, true );
}

}  // end namespace
