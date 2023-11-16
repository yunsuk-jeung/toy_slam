#include <opencv2/opencv.hpp>
#include "config.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "PointMatcher.h"
namespace toy {
PointMatcher::PointMatcher(std::string type) {}

PointMatcher::~PointMatcher() {}

int PointMatcher::process(db::ImagePyramid* imagePyramid0,
                          db::Feature*      feature0,
                          db::ImagePyramid* imagePyramid1,
                          db::Feature*      feature1) {
  const auto patch = cv::Size2i(Config::Vio::patchSize, Config::Vio::patchSize);

  std::vector<uchar> status;

  cv::calcOpticalFlowPyrLK(imagePyramid0->getPyramids(),
                           imagePyramid1->getPyramids(),
                           feature0->getKeypoints().mPoints,
                           feature1->getKeypoints().mPoints,
                           status,
                           cv::noArray(),
                           patch,
                           Config::Vio::pyramidLevel);

  cv::Mat image0 = imagePyramid0->getPyramids()[0].clone();
  cv::cvtColor(image0, image0, cv::COLOR_GRAY2BGR);

  cv::Mat image1 = imagePyramid1->getPyramids()[0].clone();
  cv::cvtColor(image1, image1, cv::COLOR_GRAY2BGR);

  const auto& uvs0 = feature0->getKeypoints().mPoints;
  const auto& uvs1 = feature1->getKeypoints().mPoints;

  for (int i = 0; i < uvs0.size(); i++) {
    if (status[i] == 0) {
      cv::line(image1, uvs0[i], uvs1[i], {0.0, 0.0, 255.0}, 1);
      cv::circle(image1, uvs1[i], 4, {0.0, 0.0, 255.0}, -1);
    }
    else {
      cv::line(image1, uvs0[i], uvs1[i], {0.0, 255.0, 0.0}, 1);
      cv::circle(image1, uvs1[i], 4, {0.0, 255.0, 0.0}, -1);
    }
  }

  cv::imshow("remove outlier", image1);
  cv::waitKey(1);
  return true;
}
}  //namespace toy