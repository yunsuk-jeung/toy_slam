#pragma once
/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <sophus/se2.hpp>

#include "patterns.h"

namespace toy {

class Patch {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Patch() = delete;
  Patch(const cv::Mat& pyr, const cv::Point2f uv)
    : mImage{pyr}
    , mUV{uv.x, uv.y} {
    prepare();
  }

protected:
  inline void prepare() {
    int   validCount = 0;
    float sum        = 0;

    Eigen::Matrix<float, 2, 3> J;

    auto& pattern = Pattern::pattern();

    for (int i = 0; i < PATTERN_SIZE; ++i) {
    }

    //mH = J.transpose();
  }

public:
  static constexpr int PATTERN_SIZE = Pattern::PATTERN_SIZE;

protected:
  const cv::Mat&        mImage;
  const Eigen::Vector2f mUV;

  Sophus::SE2f     mW;
  Eigen::Matrix3Pf mH;

  float mMeanI = 0;
  bool  valid  = false;
};
}  //namespace toy
