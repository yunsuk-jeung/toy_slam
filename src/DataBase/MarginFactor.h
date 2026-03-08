#pragma once
#include <Eigen/Dense>

namespace toy {
namespace db {
class SqrtMarginPrior {
public:

protected:
  Eigen::MatrixXd mJ;
  Eigen::MatrixXd mRes;
};

}  //namespace db
}  //namespace toy