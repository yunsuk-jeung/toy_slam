#include <iostream>
#include <functional>
#include "Slam.h"
#include <filesystem>

int main() {

  auto currPath = std::filesystem::current_path();
  auto configPath =
      currPath.parent_path().parent_path().append("configs").append("VioOnly.json");
  auto configFile = configPath.string();

  toy::SLAM::getInstance()->prepare(configFile);

  auto accCallback = [](uint64_t& ns, float* acc) {
    toy::SLAM::getInstance()->setAcc(ns, acc);
  };

  auto gyrCallback = [](uint64_t& ns, float* gyr) {
    toy::SLAM::getInstance()->setGyr(ns, gyr);
  };

  return 0;
}