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

void ObjectRecognition::extractHistogram(const Mat &src, const Mat &mask,
                                         cv::Mat &hist) {

  /// Establish the number of bins
  int bins = 30;
  int histSize[] = {bins, bins, bins};

  /// Set the ranges ( for B,G,R) )
  float range[] = {0, 256};
  const float *histRange[] = {range, range, range};

  bool uniform = true;
  bool accumulate = false;
  int channels[] = {0, 1, 2};

  calcHist(&src, 1, channels, mask, hist, 2, histSize, histRange, true, false);
  normalize(hist, hist, 0, 1, NORM_MINMAX, -1, Mat());
}

void ObjectRecognition::extractHSHistograms(const Mat &src, const Mat &mask,
                                            bool cv_normalized, Mat &hist) {

  Mat hsv;
  cvtColor(src, hsv, COLOR_BGR2HSV);

  float num_pixels;
  if (!mask.empty()) {
    num_pixels = cv::countNonZero(mask);
    // std::cout << "pixels " << num_pixels << std::endl;
  } else
    num_pixels = src.cols * src.rows;

  float h_range[] = {0, 179};
  float s_range[] = {0, 255};

  const float *hRange = {h_range};
  const float *sRange = {s_range};

  int h_bins = 50;
  int s_bins = 52;
  int histSize[] = {h_bins, s_bins};

  // hue varies from 0 to 179, saturation from 0 to 255
  float h_ranges[] = {0, 180};
  float s_ranges[] = {0, 256};

  const float *ranges[] = {h_ranges, s_ranges};
  // Use the o-th and 1-st channels
  int channels[] = {0, 1};

  bool uniform = true;
  bool accumulate = false;

  /// Calculate the histograms for the HSV images
  calcHist(&hsv, 1, channels, mask, hist, 2, histSize, ranges, true, false);

  if (cv_normalized)
    normalize(hist, hist, 0, 1, NORM_MINMAX, -1, Mat());
  else {
    for (auto i = 0; i < hist.rows; ++i) {
      for (auto j = 0; j < hist.cols; ++j) {
        hist.at<float>(i, j) = hist.at<float>(i, j) / num_pixels;
      }
    }
  }
}

void ObjectRecognition::compareHistograms(const Mat &fst, const Mat &scd) {
  float corr = compareHist(fst, scd, CV_COMP_CORREL);
  float chi = compareHist(fst, scd, CV_COMP_CHISQR);
  float inter = compareHist(fst, scd, CV_COMP_INTERSECT);
  float bha = compareHist(fst, scd, CV_COMP_BHATTACHARYYA);

  std::cout << "corr " << corr << " chi " << chi << " inter " << inter
            << " bha " << bha << std::endl;
}

void ObjectRecognition::loadData() {
  Mat rgb = imread(
      "/home/alessandro/amazon_challenge_ws/src/amazon_challenge/data/"
      "histogram_models/kyjen_squeakin_eggs_plush_puppies.png");
  Mat rgb_mask = imread(
      "/home/alessandro/amazon_challenge_ws/src/amazon_challenge/data/"
      "histogram_models/kyjen_squeakin_eggs_plush_puppies_mask.png",
      0);

  Mat hsv_0, rgb_0;
  extractHSHistograms(rgb, Mat(), false, hsv_0);
  extractHistogram(rgb, Mat(), rgb_0);

  m_hs_models.insert(
      pair<string, Mat>("kyjen_squeakin_eggs_plush_puppies", hsv_0));
  m_rgb_models.insert(
      pair<string, Mat>("kyjen_squeakin_eggs_plush_puppies", rgb_0));

  rgb = imread(
      "/home/alessandro/amazon_challenge_ws/src/amazon_challenge/data/"
      "histogram_models/first_years_take_and_toss_straw_cup.png");
  rgb_mask = imread(
      "/home/alessandro/amazon_challenge_ws/src/amazon_challenge/data/"
      "histogram_models/first_years_take_and_toss_straw_cup.png",
      0);
  /*vector<Mat> histogram2(3, Mat());
  extractHistogram(rgb, Mat(), histogram2);
  m_rgb_models.insert(pair<string, vector<Mat>>(
      "first_years_take_and_toss_straw_cup", histogram2));*/
  Mat hsv_1, rgb_1, hsv_mask_1, rgb_mask_1;
  extractHSHistograms(rgb, Mat(), true, hsv_1);
  extractHistogram(rgb, Mat(), rgb_1);

  m_hs_models.insert(
      pair<string, Mat>("first_years_take_and_toss_straw_cup", hsv_1));
  m_rgb_models.insert(
      pair<string, Mat>("first_years_take_and_toss_straw_cup", rgb_1));

  Mat test = imread(
      "/home/alessandro/amazon_challenge_ws/src/amazon_challenge/data/"
      "histogram_models/test.jpg");

  Rect balls(284, 223, 66, 68);
  Rect cups(334, 202, 62, 100);
  Rect oreo(401, 220, 93, 72);
  Rect box(335, 73, 40, 100);
  Rect glue(450, 93, 44, 92);

  std::cout << "balls \n";
  testObject(balls, test);

  std::cout << "cups \n";
  testObject(cups, test);

  std::cout << "oreo \n";
  testObject(oreo, test);

  std::cout << "box \n";
  testObject(box, test);

  std::cout << "glue \n";
  testObject(glue, test);

  // calcBackProject( &hsv, 1, channels, hist, backproj, ranges, 1, true );
}

void ObjectRecognition::testObject(Rect box, Mat &img) {
  Mat1b mask(img.rows, img.cols, uchar(0));
  rectangle(mask, box, 255, -1);
  Mat croppedImage = img(box);
  Mat hs_hist, rgb_hist;
  extractHSHistograms(croppedImage, Mat(), true, hs_hist);
  extractHistogram(croppedImage, Mat(), rgb_hist);

  auto rgb_model_0 = m_rgb_models.find("kyjen_squeakin_eggs_plush_puppies");
  auto hs_model_0 = m_hs_models.find("kyjen_squeakin_eggs_plush_puppies");

  auto rgb_model_1 = m_rgb_models.find("first_years_take_and_toss_straw_cup");
  auto hs_model_1 = m_hs_models.find("first_years_take_and_toss_straw_cup");

  std::cout << "HSV \n";
  compareHistograms(hs_hist, hs_model_0->second);
  compareHistograms(hs_hist, hs_model_1->second);
  std::cout << "RGB \n";
  compareHistograms(rgb_hist, rgb_model_0->second);
  compareHistograms(rgb_hist, rgb_model_1->second);

  float h_range[] = {0, 179};
  float s_range[] = {0, 255};
  const float *ranges[] = {h_range, s_range};
  int channels[] = {0, 1};


  Mat m0_bp, m1_bp;

  Mat hist0, hist1;
  normalize( hs_model_0->second, hist0, 0, 255, NORM_MINMAX, -1, Mat() );
  normalize( hs_model_1->second, hist1, 0, 255, NORM_MINMAX, -1, Mat() );

  calcBackProject(&croppedImage, 1, channels, hist0 , m0_bp, ranges, 1, true);
  calcBackProject(&croppedImage, 1, channels, hist1, m1_bp, ranges, 1, true);

  /// Draw the backproj
  imshow("BackProj0", m0_bp);
  imshow("BackProj1", m1_bp);
  waitKey(0);
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

void ObjectRecognition::project_histograms(const Mat &in, const Mat &hist,
                                           Mat &out) {
  cv::Mat hsv;
  cvtColor(in, hsv, COLOR_BGR2HSV);

  auto model = m_hs_models.find("first_years_take_and_toss_straw_cup");
  float h_range[] = {0, 179};
  const float *ranges = {h_range};

  calcBackProject(&hsv, 1, 0, hist, out, &ranges, 1, true);
}

}  // end namespace
