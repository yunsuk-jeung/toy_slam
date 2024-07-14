#include <iostream>
#include <filesystem>
#include <chrono>

#include <nlohmann/json.hpp>
#include <Eigen/Dense>

#include "Logger.h"
#include "EurocReader.h"

namespace io {
namespace fs    = std::filesystem;
using ImageInfo = std::pair<uint64_t, std::string>;
namespace {
void loadImage(const fs::path& path, std::deque<ImageInfo>& infos) {
  for (const auto& entry : fs::directory_iterator(path)) {
    uint64_t    ns  = std::stoull(entry.path().stem().string());
    std::string loc = entry.path().string();
    infos.push_back({ns, loc});
  }
  std::sort(infos.begin(), infos.end(), [](const ImageInfo& a, const ImageInfo& b) {
    return a.first < b.first;
  });
}
}  //namespace

EurocReader::EurocReader() {
  //main
  mImage0Type = 0;
  //sub
  mImage1Type = 1;
}

EurocReader::~EurocReader() {}

void EurocReader::openDirectory(std::string configFile,
                                std::string dataDir,
                                bool        uploadMemory) {
  parseConfig(configFile);

  auto mav0 = fs::path(dataDir);
  if (!fs::exists(mav0)) {
    LOGE("mav0 is missing, input path : {}", mav0.string());
    throw std::runtime_error("path dosent' exist");
  }
  LOGI("Opening dataset : {}", mav0.string());
  mav0.append("mav0");

  auto camPath0 = mav0;
  camPath0.append("cam0").append("data");
  loadImage(camPath0, mImageInfos0);

  auto    front0 = mImageInfos0.front();
  cv::Mat image0 = cv::imread(front0.second, cv::IMREAD_GRAYSCALE);
  LOGI("cam0 type : {} / format : {}", mImage0Type, image0.type());

  auto camPath1 = mav0;
  camPath1.append("cam1").append("data");
  loadImage(camPath1, mImageInfos1);

  auto    front1 = mImageInfos1.front();
  cv::Mat image1 = cv::imread(front1.second, cv::IMREAD_GRAYSCALE);
  LOGI("cam1 type : {} / format : {}", mImage1Type, image1.type());

  syncStereo();

  mUploadMemory = uploadMemory;

  if (mUploadMemory)
    load();
  else
    loadAsync();

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

void EurocReader::getInfos(CameraInfo& cam0, CameraInfo& cam1) {
  cam0 = mCamInfo0;
  cam1 = mCamInfo1;
}

bool EurocReader::getImages(int&      type0,
                            uint64_t& ns0,
                            cv::Mat&  image0,
                            int&      type1,
                            uint64_t& ns1,
                            cv::Mat&  image1) {
  if (mImageDeque0.empty()) {
    //LOGW("image queue is empty");
    return false;
  }

  type0  = mImage0Type;
  ns0    = mImageNsDeque0.front();
  image0 = mImageDeque0.front();

  type1  = mImage1Type;
  ns1    = mImageNsDeque1.front();
  image1 = mImageDeque1.front();

  {
    std::unique_lock<std::mutex> uniqueLock(mLoadLock);
    mImageNsDeque0.pop_front();
    mImageDeque0.pop_front();
    mImageNsDeque1.pop_front();
    mImageDeque1.pop_front();
  }

  return true;
}

void EurocReader::syncStereo() {
  auto ns0 = mImageInfos0.front().first;
  auto ns1 = mImageInfos1.front().first;

  if (ns0 > ns1) {
    while (ns0 != ns1) {
      mImageInfos1.pop_front();
      ns1 = mImageInfos1.front().first;
    }
  }
  else {
    while (ns0 != ns1) {
      mImageInfos0.pop_front();
      ns0 = mImageInfos0.front().first;
    }
  }

  for (size_t i = 0; i < mImageInfos0.size(); i++) {
    if (mImageInfos0[i].first != mImageInfos1[i].first) {
      throw std::runtime_error("dataset is strange");
    }
  }
}

void EurocReader::parseConfig(std::string configFile) {
  std::ifstream jsonFile(configFile);

  if (!jsonFile.is_open()) {
    LOGE("missing config file : {}", configFile);
    return;
  }
  nlohmann::json jsonObject;
  jsonFile >> jsonObject;

  {
    auto cam0Json             = jsonObject["cam0"];
    mCamInfo0.id              = 0;
    mCamInfo0.w               = cam0Json["resolution"][0].get<int>();
    mCamInfo0.h               = cam0Json["resolution"][1].get<int>();
    mCamInfo0.cameraModel     = cam0Json["cameraModel"];
    mCamInfo0.intrinsics      = cam0Json["intrinsics"].get<std::vector<float>>();
    mCamInfo0.distortionModel = cam0Json["distortionModel"];

    auto distortion = cam0Json["distortions"].get<std::vector<float>>();
    memcpy(mCamInfo0.distortions.data(),
           distortion.data(),
           sizeof(float) * distortion.size());

    std::vector<float> bMcRmjor = cam0Json["Mbc"].get<std::vector<float>>();
    Eigen::Matrix4f    Mbc;
    memcpy(Mbc.data(), bMcRmjor.data(), sizeof(float) * 16);
    Mbc.transposeInPlace();
    memcpy(mCamInfo0.Mbc.data(), Mbc.data(), sizeof(float) * 16);
  }
  {
    auto cam1Json             = jsonObject["cam1"];
    mCamInfo1.id              = 1;
    mCamInfo1.w               = cam1Json["resolution"].get<std::vector<int>>()[0];
    mCamInfo1.h               = cam1Json["resolution"].get<std::vector<int>>()[1];
    mCamInfo1.cameraModel     = cam1Json["cameraModel"];
    mCamInfo1.intrinsics      = cam1Json["intrinsics"].get<std::vector<float>>();
    mCamInfo1.distortionModel = cam1Json["distortionModel"];

    auto distortion = cam1Json["distortions"].get<std::vector<float>>();
    memcpy(mCamInfo1.distortions.data(),
           distortion.data(),
           sizeof(float) * distortion.size());

    std::vector<float> bMcRmjor = cam1Json["Mbc"].get<std::vector<float>>();
    Eigen::Matrix4f    Mbc;
    memcpy(Mbc.data(), bMcRmjor.data(), sizeof(float) * 16);
    Mbc.transposeInPlace();
    memcpy(mCamInfo1.Mbc.data(), Mbc.data(), sizeof(float) * 16);
  }
}

void EurocReader::load() {}

void EurocReader::loadAsync() {
  mLoading = true;

  auto loadFunc = [&]() {
    while (mLoading) {
      if (mImageInfos0.empty() || mImageInfos1.empty())
        break;

      auto&    info0  = mImageInfos0.front();
      uint64_t ns0    = info0.first;
      cv::Mat  image0 = cv::imread(info0.second, cv::IMREAD_GRAYSCALE);

      auto&    info1  = mImageInfos1.front();
      uint64_t ns1    = info1.first;
      cv::Mat  image1 = cv::imread(info1.second, cv::IMREAD_GRAYSCALE);

      {
        std::unique_lock<std::mutex> uniqueLock(mLoadLock);
        mImageNsDeque0.push_back(ns0);
        mImageDeque0.push_back(image0);

        mImageNsDeque1.push_back(ns1);
        mImageDeque1.push_back(image1);
      }

      mImageInfos0.pop_front();
      mImageInfos1.pop_front();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  };

  mLoadThread = std::thread(loadFunc);
}

}  //namespace io