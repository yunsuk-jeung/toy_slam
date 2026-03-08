#pragma once
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <fmt/format.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
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
  fs::path saveDir = currDir.parent_path()
                       .parent_path()
                       .parent_path()
                       .append("python")
                       .append("data")
                       .append(folder);

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

template <typename Matrix>
void drawSparseMatrix(const std::string& name, const Matrix& mat, const int key = 0) {
  cv::Mat matrix;

  matrix = cv::Mat::zeros(cv::Size(mat.cols(), mat.rows()), CV_8UC1);

  for (int i = 0; i < mat.rows(); ++i) {
    for (int j = 0; j < mat.cols(); ++j) {
      if (abs(mat(i, j)) > 1e-7 ) {
        matrix.at<cv::uint8_t>(i, j) = cv::uint8_t(188);
      }
    }
  }

  if (matrix.cols < 640 || matrix.rows < 640) {
    int a = 640 / matrix.cols;
    int b = 640 / matrix.rows;

    auto c = a > b ? a : b;

    cv::Size2i size = cv::Size2i(matrix.cols * c, matrix.rows * c);
    cv::resize(matrix, matrix, size, 0, 0, cv::INTER_NEAREST_EXACT);
  }

  cv::imshow(name, matrix);
  cv::waitKey(key);
}

}  //namespace debug
}  //namespace toy
#endif