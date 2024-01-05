#pragma once

#include <vector>
#include <map>
#include <Eigen/Dense>
#include "Frame.h"
#include "macros.h"
#include "SqrtMarginalizer.h"

namespace toy {
class SqrtMarginalizer;
class SqrtMarginalizationCost {
public:
  USING_SMART_PTR(SqrtMarginalizationCost);

  SqrtMarginalizationCost(SqrtMarginalizer*            marginalizer,
                          std::vector<db::Frame::Ptr>& remainFrames)
    : mSqrtMarginalizer{marginalizer}
    , mJ{marginalizer->getJ()}
    , mRes{marginalizer->getRes()} {
    mRemainFrames.swap(remainFrames);
    mRows = mRemainFrames.size() * db::Frame::PARAMETER_SIZE;

    mInitialParameters.resize(mRows);
    mCurrentParameters.resize(mRows);

    updateParameters();
    mInitialParameters = mCurrentParameters;
  }

  ~SqrtMarginalizationCost() {
    mSqrtMarginalizer = nullptr;
    mRemainFrames.clear();
  }

  double linearize() {
    Eigen::VectorXd delta = mCurrentParameters - mInitialParameters;
    auto&           J     = mSqrtMarginalizer->J();
    auto&           Res   = mSqrtMarginalizer->Res();

    return delta.transpose() * J.transpose() * (0.5 * J * delta + Res);
  }

  void addToHessian(Eigen::MatrixXd& H, Eigen::VectorXd& B) {
    const auto& POSE_SIZE = db::Frame::PARAMETER_SIZE;
    H.block(0, 0, mRows, mRows) += mJ.transpose() * mJ;
    B.segment(0, mRows) -= mJ.transpose() * mRes;
  }

protected:
  void updateParameters() {
    Eigen::Index row = 0;
    for (auto& f : mRemainFrames) {
      mCurrentParameters.segment(row, db::Frame::PARAMETER_SIZE) = f->toParameter();
      row += db::Frame::PARAMETER_SIZE;
    }
  }

protected:
  SqrtMarginalizer*           mSqrtMarginalizer;
  std::vector<db::Frame::Ptr> mRemainFrames;
  Eigen::VectorXd             mInitialParameters;
  Eigen::VectorXd             mCurrentParameters;
  Eigen::MatrixXd&            mJ;
  Eigen::VectorXd&            mRes;
  size_t                      mRows;
};
}  //namespace toy