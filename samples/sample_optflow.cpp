#include <iostream>
#include <filesystem>

#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

#include "ImagePyramid.h"
#include "ToyLogger.h"
#include "IoUtil.h"
#include "PointExtractor.h"

using namespace toy;
auto main() -> int {

  ToyLogI("Starting opticalflow module ");

  std::string           currCpp = __FILE__;
  std::filesystem::path path(currCpp);
  std::filesystem::path resourcePath = path.parent_path().append("resources");
  ToyLogD("Resource directory: {}", resourcePath.string());

  auto pngs = io::util::getFiles(resourcePath, ".png");
  ToyLogI("png size : {}", pngs.size());

  std::vector<db::ImagePyramid> pyramids;
  for (const auto& png : pngs) {
    cv::Mat image = cv::imread(png.string(), CV_LOAD_IMAGE_GRAYSCALE);
    pyramids.push_back({ImageType::MAIN, image});
  }

  toy::PointTracker extractor;

  std::vector<std::vector<cv::Point2f>> pointss;
  pointss.resize(pyramids.size());

  for (auto i = 0; i < pyramids.size(); i++) {
    auto  img    = pyramids[i].getGray();
    auto& points = pointss[i];
    extractor.process(img, points);
  }

  return 0;
}