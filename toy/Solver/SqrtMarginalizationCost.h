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
    , mJ{J}
    , mRes{Res} {
    //mCurrentParameters.resize(rows);
    //mInitialParameters.resize(rows);

    //need one more jacobian for fixed frame

    //updateParameters();
    //mInitialParameters = mCurrentParameters;
  }

  ~SqrtMarginalizationCost() = default;

  double linearize() {
    //updateParameters();
    //auto rows = mFrames.size() * db::Frame::PARAMETER_SIZE;

    Eigen::VectorXd delta = getDelta();
    //Eigen::VectorXd delta = (mCurrentParameters - mInitialParameters).head(rows);
    return delta.transpose() * mJ.transpose() * (0.5 * mJ * delta + mRes);
  }

  void addToHessian(Eigen::MatrixXd& H, Eigen::VectorXd& B) {
    //updateParameters();
    auto rows = mFrames.size() * db::Frame::PARAMETER_SIZE;
    //Eigen::VectorXd delta = (mCurrentParameters - mInitialParameters).head(rows);
    Eigen::VectorXd delta = getDelta();

    auto cols = mJ.cols();  //same as parameter rows
    H.block(0, 0, cols, cols) += mJ.transpose() * mJ;
    B.segment(0, cols) -= mJ.transpose() * (mJ * delta + mRes);
  }

  void addToQRJacobian(Eigen::MatrixXd& Q2t_J, Eigen::VectorXd& Q2t_C, size_t& startRow) {
    auto delta = getDelta();

    Q2t_J.block(startRow, 0, mJ.rows(), mJ.cols()) = mJ;
    Q2t_C.segment(startRow, mJ.rows())             = mJ * delta + mRes;

    startRow += mJ.rows();
  }

protected:
  //void updateParameters() {
  //  Eigen::Index row = 0;
  //  for (auto& f : mFrames) {
  //    mCurrentParameters.segment(row, db::Frame::PARAMETER_SIZE) = f->toParameter();
  //    if (f->fixed()) {
  //      mJ.block(0, row, mJ.rows(), db::Frame::PARAMETER_SIZE).setZero();
  //    }
  //    row += db::Frame::PARAMETER_SIZE;
  //  }
  //}

  Eigen::VectorXd getDelta() {
    Eigen::VectorXd out(mJ.cols());
    Eigen::Index    row = 0;

    for (auto& f : mFrames) {
      out.segment(row, db::Frame::PARAMETER_SIZE) = f->getDelta();
      row += db::Frame::PARAMETER_SIZE;
    }
    return out;
  }

protected:
  const std::vector<db::Frame::Ptr>& mFrames;
  //Eigen::VectorXd                    mInitialParameters;
  //Eigen::VectorXd                    mCurrentParameters;

  //const Eigen::MatrixXd& mJ_marg;
  //const Eigen::VectorXd& mRes_marg;

  Eigen::MatrixXd mJ;
  Eigen::MatrixXd mRes;

public:
  auto  rows() { return mJ.rows(); }
  auto& J() { return mJ; }
  auto& Res() { return mRes; }
};
}  //namespace toy