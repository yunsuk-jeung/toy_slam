#pragma once

#include <vector>
#include <Eigen/Dense>
#include "macros.h"
#include "Frame.h"

namespace toy {
class SqrtMarginalizationCost {
public:
  USING_SMART_PTR(SqrtMarginalizationCost);

  SqrtMarginalizationCost() = delete;
  SqrtMarginalizationCost(const std::vector<db::Frame::Ptr>& frames,
                          const Eigen::MatrixXd&             J,
                          const Eigen::VectorXd&             Res)
    : mFrames{frames}
    , mJ{J}
    , mRes{Res} {
    
    auto rows = frames.size() * db::Frame::PARAMETER_SIZE;
    mCurrentParameters.resize(rows);
    mInitialParameters.resize(rows);

    updateParameters();
  }

  ~SqrtMarginalizationCost() = default;

  double linearize() {
    auto rows = mFrames.size() * db::Frame::PARAMETER_SIZE;

    Eigen::VectorXd delta = (mCurrentParameters - mInitialParameters).head(rows);
    return delta.transpose() * mJ.transpose() * (0.5 * mJ * delta + mRes);
  }

  void addToHessian(Eigen::MatrixXd& H, Eigen::VectorXd& B) {
    auto cols = mJ.cols();  //same as parameter rows

    H.block(0, 0, cols, cols) += mJ.transpose() * mJ;
    B.segment(0, cols) -= mJ.transpose() * mRes;
  }

protected:
  void updateParameters() {
    Eigen::Index row = 0;
    for (auto& f : mFrames) {
      mCurrentParameters.segment(row, db::Frame::PARAMETER_SIZE) = f->toParameter();
      row += db::Frame::PARAMETER_SIZE;
    }
  }

protected:
  const std::vector<db::Frame::Ptr>& mFrames;
  Eigen::VectorXd                    mInitialParameters;
  Eigen::VectorXd                    mCurrentParameters;

  const Eigen::MatrixXd& mJ;
  const Eigen::VectorXd& mRes;
};
}  //namespace toy