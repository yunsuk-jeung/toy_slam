#include <iostream>
#include <filesystem>

#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

#include "ImagePyramid.h"
#include "PointTrackingModule.h"
#include "Logger.h"
#include "IoUtil.h"

using namespace toy;
auto main() -> int {
  spdlog::set_level(spdlog::level::debug);

  LOGI("Starting opticalflow module ");

  std::string           currCpp = __FILE__;
  std::filesystem::path path(currCpp);
  std::filesystem::path resourcePath = path.parent_path().append("resources");

  LOGD("Resource directory: {}", resourcePath.string());

  auto pngs = io::util::getFiles(resourcePath, ".png");

  LOGI("png size : {}", pngs.size());

  std::vector<db::ImagePyramid> pyramids;

  for (const auto& png : pngs) {
    cv::Mat image = cv::imread(png.string(), CV_LOAD_IMAGE_GRAYSCALE);
    pyramids.push_back({ImageType::MAIN, image});
  }

  for (auto& pyramid : pyramids) {
    auto img = pyramid.getOrigin();
    cv::imshow("img ", img);
    cv::waitKey();
  }

  return 0;
}