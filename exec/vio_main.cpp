#include <iostream>
#include <functional>

#include <tbb/concurrent_unordered_map.h>
#include <Eigen/Dense>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "Slam.h"

#include <ceres/ceres.h>
using Map = tbb::concurrent_unordered_map<int, int, std::hash<int>>;

int main() {

  toy::SLAM::getInstance()->init(std::string("wtf"));

  auto accCallback = [](uint64_t& ns, float* acc) {
    toy::SLAM::getInstance()->setAcc(ns, acc);
  };

  auto gyrCallback = [](uint64_t& ns, float* gyr) {
    toy::SLAM::getInstance()->setGyr(ns, gyr);
  };

  nlohmann::json ex1 = nlohmann::json::parse(R"(
  {
    "pi": 3.141,
    "happy": true
  }
)");
  spdlog::info("Welcome to spdlog!");

  Map map;

  int i = 0;

  map.emplace(i, i);
  i++;
  map.emplace(i, i);
  i++;
  map.emplace(i, i);
  i++;
  map.emplace(i, i);
  map.emplace(i, i);
  map.emplace(i, i);
  map.emplace(i, i);

  Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
  std::cout << I << std::endl;

  for (auto aa = map.begin(); aa != map.end(); aa++) {
    std::cout << aa->first << " / " << aa->second << std::endl;
  }

  for (auto& [key, val] : map) {
    std::cout << key << " / " << val << std::endl;
  }

  cv::Mat image(480, 640, CV_8UC3);
  cv::putText(image, "OPENCV_TEST", {0, 100}, 0, 2, {255.0, 0.0, 0.0}, 1);
  cv::imshow("sample", image);
  cv::waitKey(1000);


  return 0;
}