#pragma once
#include <vector>
#include <Eigen/Dense>
#include "macros.h"
#include "Frame.h"
#include "ToyAssert.h"

namespace toy {
class SqrtMarginalizationCost {
public:
  USING_SMART_PTR(SqrtMarginalizationCost);

  SqrtMarginalizationCost()  = default;
  ~SqrtMarginalizationCost() = default;

  void setRemainIds(std::vector<size_t>& ids) { mRemainIds.swap(ids); }

  void setFrames(std::vector<std::shared_ptr<db::Frame>>& frames) {
    mFrames = frames;
    auto rows     = mFrames.size() * db::Frame::PARAMETER_SIZE;

    mInitialParameters.resize(rows);
    mCurrentParameters.resize(rows);

    updateParameters();
    mInitialParameters = mCurrentParameters;
  }

  double linearize() {
    auto rows     = mRemainIds.size() * db::Frame::PARAMETER_SIZE;

    Eigen::VectorXd delta = (mCurrentParameters - mInitialParameters).head(rows);
    return delta.transpose() * mJ.transpose() * (0.5 * mJ * delta + mRes);
  }

  void addToHessian(Eigen::MatrixXd& H, Eigen::VectorXd& B) {
    auto cols = mJ.cols();  //same as parameter rows

    H.block(0, 0, cols, cols) += mJ.transpose() * mJ;
    B.segment(0, cols) -= mJ.transpose() * mRes;
  }

  void marginalize(Eigen::VectorXi& indices, Eigen::MatrixXd& J, Eigen::VectorXd& Res) {
    const Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>> permutation(
      indices);

    J.applyOnTheRight(permutation);
    size_t margRank       = 0;
    size_t validBlockRows = 0;

    decomposeWithQR(J, Res, db::Frame::PARAMETER_SIZE, margRank, validBlockRows);

    ToyLogD("after qr {}", J);

    mJ   = J.block(margRank,
                 db::Frame::PARAMETER_SIZE,
                 validBlockRows,
                 indices.cols() - db::Frame::PARAMETER_SIZE);
    mRes = Res.segment(margRank, validBlockRows);
  }

protected:
  void updateParameters() {
    Eigen::Index row = 0;
    for (auto& f : mFrames) {
      mCurrentParameters.segment(row, db::Frame::PARAMETER_SIZE) = f->toParameter();
      row += db::Frame::PARAMETER_SIZE;
    }
  }

  void decomposeWithQR(Eigen::MatrixXd& J,
                       Eigen::VectorXd& Res,
                       const size_t&    marginBlockSize,
                       size_t&          marginRank,
                       size_t&          validBlockRows) {}

protected:
  std::vector<db::Frame::Ptr> mFrames;
  Eigen::VectorXd             mInitialParameters;
  Eigen::VectorXd             mCurrentParameters;

  Eigen::MatrixXd mJ;
  Eigen::VectorXd mRes;

  std::vector<size_t> mRemainIds;

public:
  const Eigen::MatrixXd& J() const { return mJ; }
  Eigen::MatrixXd&       getJ() { return mJ; }
  const Eigen::VectorXd& Res() const { return mRes; }
  Eigen::VectorXd&       getRes() { return mRes; }
  std::vector<size_t>&   getRemainIds() { return mRemainIds; }
};
}  //namespace toy