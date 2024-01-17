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
#include "ToyAssert.h"
#include "ImageUtil.h"
#include "patterns.h"

namespace toy {

class Patch {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Patch() = delete;
  Patch(const cv::Mat& pyr, const cv::Point2f& uv)
    : mPattern(Pattern::pattern)
    , mMeanI{0}
    , mValid{false}
    , mImage(pyr)
    , mUv{uv.x, uv.y} {
    prepareInverseComposition();
    std::cout << mWarp.matrix() << std::endl;
  }

  bool match(const cv::Mat& targetImage, cv::Point2f& uv) {
    bool valid          = true;
    mWarp.translation() = Eigen::Vector2f{uv.x, uv.y};

    //cv::Mat colorOri = mImage.clone();
    //cv::cvtColor(colorOri, colorOri, CV_GRAY2BGR);
    //cv::circle(colorOri, cv::Point2f(mUv.x(), mUv.y()), 5, {255, 0, 0}, -1);
    //cv::imshow("origin", colorOri);

    //cv::Mat colorDst = targetImage.clone();
    //cv::cvtColor(colorDst, colorDst, CV_GRAY2BGR);
    //cv::circle(colorDst, uv, 5, {0, 0, 255}, -1);

    for (int i = 0; valid && i < MAX_ITERATION; ++i) {
      Eigen::Matrix2Pf warpedPattern = mWarp.so2().matrix() * mPattern;
      warpedPattern.colwise() += mWarp.translation();

      Eigen::VectorPf res;

      valid &= calculateResidual(targetImage, warpedPattern, res);

      if (!valid) {
        continue;
      }

      Eigen::Vector3f del = -mH_inv_Jt * res;

      valid &= del.array().isFinite().all();
      valid &= del.lpNorm<Eigen::Infinity>() < 1e6;

      ToyLogD("iter {} -- res {} del {}", i, res.norm(), del);

      if (!valid) {
        continue;
      }

      mWarp *= Sophus::SE2f::exp(del);
      //cv::circle(colorDst,
      //           cv::Point2f(mWarp.translation().x(), mWarp.translation().y()),
      //           5,
      //           {255, 0, 0},
      //           -1);
      //cv::imshow("target", colorDst);
      //cv::waitKey();

      valid &= util::inBounds(targetImage, mWarp.translation(), FILTER_MARGIN);
    }

    if (valid) {
      uv.x = mWarp.translation().x();
      uv.y = mWarp.translation().y();
    }
    return valid;
  }

protected:
  inline void prepareInverseComposition() {
    /*    cost = I - avg(I)    */

    int              validCount = 0;
    float            sum        = 0;
    Eigen::Vector3f  J_I_se2_sum(0, 0, 0);
    Eigen::MatrixP3f J_I_se2 = Eigen::MatrixP3f::Zero();

    Eigen::Matrix<float, 2, 3> J_uv_se2;  //jacobian for pixel
    J_uv_se2.setIdentity();

    for (int i = 0; i < PATTERN_SIZE; ++i) {
      Eigen::Vector2f uv1 = mUv + mPattern.col(i);

      J_uv_se2(0, 2) = -mPattern(1, i);
      J_uv_se2(1, 2) = mPattern(0, i);

      if (util::inBounds(mImage, uv1, 2)) {
        Eigen::Vector3f grad = util::interpolateGradient(mImage, uv1, util::LINEAR);

        mIs[i]               = grad[0];
        sum += grad[0];

        J_I_se2.row(i) = grad.tail(2).transpose() * J_uv_se2;
        J_I_se2_sum += J_I_se2.row(i);
        ++validCount;
      }
      else {
        mIs[i] = -1;
      }
    }
    mMean                = sum / validCount;
    const float mean_inv = 1.0f / mMean;

    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (mIs[i] >= 0) {
        J_I_se2.row(i) -= J_I_se2_sum.transpose() * mIs[i] / sum;
        mIs[i] *= mean_inv;
      }
    }

    J_I_se2 *= mean_inv;

    Eigen::MatrixP3f& J  = J_I_se2;
    Eigen::Matrix3Pf  Jt = J.transpose();
    Eigen::Matrix3f   H  = Jt * J;

    Eigen::Matrix3f H_inv;
    H_inv.setIdentity();
    H.ldlt().solveInPlace(H_inv);
    mH_inv_Jt = H_inv * Jt;

    mValid    = mMean > std::numeric_limits<float>::epsilon()
             && mH_inv_Jt.array().isFinite().all() && mIs.array().isFinite().all();
  }

  bool calculateResidual(const cv::Mat&   target,
                         Eigen::Matrix2Pf warpedPattern,
                         Eigen::VectorPf& res) {
    float sum        = 0;
    int   validCount = 0;
    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (!util::inBounds<float>(target, warpedPattern.col(i), 2)) {
        res[i] = -1;
        continue;
      }
      res[i] = util::interpolate<float>(target, warpedPattern.col(i), util::LINEAR);
      sum += res[i];
      validCount++;
    }

    if (sum < std::numeric_limits<float>::epsilon()) {
      res.setZero();
      return false;
    }

    int validResidualCount = 0;

    for (int i = 0; i < PATTERN_SIZE; i++) {
      if (res[i] >= 0 && mIs[i] >= 0) {
        const float val = res[i];
        res[i]          = validCount * val / sum - mIs[i];
        validResidualCount++;
      }
      else {
        res[i] = 0;
      }
    }

    return validResidualCount > PATTERN_SIZE / 2;
  }

public:
  const bool& isValid() const { return mValid; }

protected:
  const Eigen::Matrix2Pf& mPattern;
  const cv::Mat&          mImage;
  const Eigen::Vector2f   mUv;

  Eigen::VectorPf mIs;
  float           mMean;

  Sophus::SE2f     mWarp;
  Eigen::Matrix3Pf mH_inv_Jt;

  float mMeanI;
  bool  mValid;

  static constexpr int PATTERN_SIZE  = Pattern::PATTERN_SIZE;
  static constexpr int MAX_ITERATION = 5;
  static constexpr int FILTER_MARGIN = 2;
};
}  //namespace toy
