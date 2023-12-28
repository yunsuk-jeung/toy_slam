#pragma once
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <fmt/format.h>
#include "IoUtil.h"
#include "ToyLogger.h"

//YSTODO to camke list
#define DEBUG_MATRIX

#ifdef DEBUG_MATRIX
//#define DEBUG_MATRIX0

namespace toy {
namespace debug {
template <typename Matrix>
void saveMatrix(const std::string& name,
                int                _idx,
                const Matrix&      mat,
                const std::string& folder = "default") {
  namespace fs = std::filesystem;
  fs::path currDir(std::string(__FILE__));
  fs::path
    saveDir = currDir.parent_path().parent_path().parent_path().append("python").append(
      "data").append(folder);

  io::util::createDirectory(saveDir);

  std::string idx      = fmt::format("{:05}", _idx);
  std::string saveName = name + "_" + idx + ".csv";
  saveDir.append(saveName);

  std::ofstream file(saveDir.string());
  if (file.is_open()) {
    file << mat;
  }
  file.close();

  ToyLogD("saved {}, {} x {}", saveDir.string(), mat.rows(), mat.cols());
};
}  //namespace debug
}  //namespace toy
#endif