#pragma once

#include <vector>
#include <Eigen/Dense>
#include "macros.h"

namespace toy {
namespace db {
class Frame;
}
class SqrtMarginalizationCost;
class SqrtMarginalizer {
public:
  USING_SMART_PTR(SqrtMarginalizer);
  SqrtMarginalizer()  = default;
  ~SqrtMarginalizer() = default;

  void setFrames(std::vector<std::shared_ptr<db::Frame>>& frames);
  std::shared_ptr<SqrtMarginalizationCost> createMarginCost();

  void marginalize(Eigen::VectorXi& indices, Eigen::MatrixXd& J, Eigen::VectorXd& Res);

protected:
  void decomposeWithQR(Eigen::MatrixXd& J,
                       Eigen::VectorXd& Res,
                       const size_t&    marginBlockSize,
                       size_t&          marginRank,
                       size_t&          validBlockRows);

protected:

  Eigen::MatrixXd mJ;
  Eigen::VectorXd mRes;

  std::vector<std::shared_ptr<db::Frame>> mFrames;

public:
  const Eigen::MatrixXd& J() const { return mJ; }
  Eigen::MatrixXd&       getJ() { return mJ; }
  const Eigen::VectorXd& Res() const { return mRes; }
  Eigen::VectorXd&       getRes() { return mRes; }
};
}  //namespace toy