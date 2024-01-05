#pragma once
#include <Eigen/Dense>

namespace toy {
class SqrtMarginalizer {
public:
  SqrtMarginalizer()  = default;
  ~SqrtMarginalizer() = default;

protected:
  Eigen::MatrixXd mJ;
  Eigen::VectorXd mC;

public:
  const Eigen::MatrixXd& J() const { return mJ; }
  Eigen::MatrixXd&       getJ() { return mJ; }
  const Eigen::VectorXd& C() const { return mC; }
  Eigen::VectorXd&       getC() { return mC; }
};
}  //namespace toy