#include <iostream>
#include <filesystem>

#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

#include "ImagePyramid.h"
#include "ToyLogger.h"
#include "IoUtil.h"
#include "Frame.h"
#include "PointTracker.h"
#include "Camera.h"
//#include "patch.h"
//#include "ImageUtil.h"
using namespace toy;

//void drawKeyPoint(db::Frame::Ptr f) {
//  auto& pyramid0   = f->getImagePyramid(0)->getPyramids();
//  auto& keyPoints0 = f->getFeature(0)->getKeypoints();
//  auto& uvs0       = keyPoints0.mUVs;
//
//  cv::Mat image = f->getImagePyramid(0)->getOrigin().clone();
//  cv::cvtColor(image, image, CV_GRAY2BGR);
//
//  for (auto& uv : uvs0) {
//    cv::circle(image, uv, 5, {255, 0, 0}, -1);
//  }
//
//  cv::imshow(std::to_string(f->id()), image);
//  cv::waitKey();
//}

auto main() -> int {
  //util::foo();

  ToyLogger::init();
  ToyLogI("Starting opticalflow module ");

  std::string           currCpp = __FILE__;
  std::filesystem::path path(currCpp);
  std::filesystem::path resourcePath = path.parent_path().append("resources");
  ToyLogD("Resource directory: {}", resourcePath.string());

  auto pngs = io::util::getFiles(resourcePath, ".png");
  ToyLogI("png size : {}", pngs.size());

  /*std::vector<db::ImagePyramidSet> pyramids;*/

  //Config::Vio::showExtraction   = true;
  //Config::Vio::showMonoTracking = true;

  std::vector<db::Frame::Ptr> frames;

  uint64_t ns = 0;
  for (const auto& png : pngs) {
    cv::Mat   image = cv::imread(png.string(), CV_LOAD_IMAGE_GRAYSCALE);
    ImageData data0{0, CV_8UC1, ns, image.data, image.cols, image.rows};
    ImageData data1{-1, CV_8UC1, ns, nullptr, image.cols, image.rows};

  db::ImagePyramid::Uni    pyr0  = std::make_unique<db::ImagePyramid>(data0);
  db::ImagePyramid::Uni    pyr1  = std::make_unique<db::ImagePyramid>(data1);
  db::ImagePyramidSet::Ptr set   = std::make_shared<db::ImagePyramidSet>(pyr0, pyr1);
  db::Frame::Ptr           frame = std::make_shared<db::Frame>(set);
  frames.push_back(frame);

  std::vector<float> Mbc(16);
  Eigen::Matrix4f    I = Eigen::Matrix4f::Identity();
  memcpy(Mbc.data(), I.data(), sizeof(float) * 16);
  float f  = image.rows;
  float cx = float(image.cols) / 2.0f;
  float cy = float(image.rows) / 2.0f;

  CameraInfo camInfo0;
  camInfo0.id              = 0;
  camInfo0.w               = image.cols;
  camInfo0.h               = image.rows;
  camInfo0.cameraModel     = 0;
  camInfo0.intrinsics      = {f, f, cx, cy};
  camInfo0.distortionModel = 0;
  camInfo0.distortions     = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  camInfo0.Mbc             = Mbc;
  auto camInfo1            = camInfo0;
  camInfo1.id              = -1;

  Camera* cam0 = CameraFactory::createCamera(&camInfo0);
  Camera* cam1 = CameraFactory::createCamera(&camInfo1);

  frame->setCameras(cam0, cam1);
  }

  auto* pointTracker = new toy::PointTracker("Fast.PatchOpticalFlow");

  auto it         = frames.begin();
  auto beforeLast = std::prev(frames.end(), 2);

  pointTracker->process(nullptr, (*it).get());
  cv::waitKey();

  for (; it != beforeLast; it++) {
    auto next = std::next(it, 1);
    pointTracker->process((*it).get(), (*next).get());
    int key = cv::waitKey();
    if (key == 27)
      break;
  }

  return 0;
}