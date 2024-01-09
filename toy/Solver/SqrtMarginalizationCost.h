#pragma once

#include <vector>
#include <Eigen/Dense>
#include "macros.h"
#include "Frame.h"
#include "ToyLogger.h"

namespace toy {
class SqrtMarginalizationCost {
public:
  USING_SMART_PTR(SqrtMarginalizationCost);

  SqrtMarginalizationCost() = delete;
  SqrtMarginalizationCost(const std::vector<db::Frame::Ptr>& frames,
                          const Eigen::MatrixXd&             J,
                          const Eigen::VectorXd&             Res)
    : mFrames{frames}
    , mJ_marg{J}
    , mRes_marg{Res} {
    auto rows = frames.size() * db::Frame::PARAMETER_SIZE;
    mCurrentParameters.resize(rows);
    mInitialParameters.resize(rows);

    //need one more jacobian for fixed frame
    mJ = mJ_marg;
    updateParameters();
    mInitialParameters = mCurrentParameters;
  }

  ~SqrtMarginalizationCost() = default;

  double linearize() {
    updateParameters();
    auto            rows  = mFrames.size() * db::Frame::PARAMETER_SIZE;
    Eigen::VectorXd delta = (mCurrentParameters - mInitialParameters).head(rows);
    return delta.transpose() * mJ_marg.transpose() * (0.5 * mJ_marg * delta + mRes_marg);
  }

  void addToHessian(Eigen::MatrixXd& H, Eigen::VectorXd& B) {
    updateParameters();
    auto            rows  = mFrames.size() * db::Frame::PARAMETER_SIZE;
    Eigen::VectorXd delta = (mCurrentParameters - mInitialParameters).head(rows);

    auto cols = mJ_marg.cols();  //same as parameter rows
    H.block(0, 0, cols, cols) += mJ.transpose() * mJ;
    B.segment(0, cols) -= mJ.transpose() * (mJ_marg * delta + mRes_marg);
  }

protected:
  void updateParameters() {
    Eigen::Index row = 0;
    for (auto& f : mFrames) {
      mCurrentParameters.segment(row, db::Frame::PARAMETER_SIZE) = f->toParameter();
      if (f->fixed()) {
        mJ.block(0, row, mJ.rows(), db::Frame::PARAMETER_SIZE).setZero();
      }
      row += db::Frame::PARAMETER_SIZE;
    }
  }

protected:
  const std::vector<db::Frame::Ptr>& mFrames;
  Eigen::VectorXd                    mInitialParameters;
  Eigen::VectorXd                    mCurrentParameters;

  const Eigen::MatrixXd& mJ_marg;
  const Eigen::VectorXd& mRes_marg;

  Eigen::MatrixXd mJ;
  Eigen::MatrixXd mRes;
};
}  //namespace toy