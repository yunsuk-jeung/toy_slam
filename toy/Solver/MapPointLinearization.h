#pragma once
#include <memory>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include "Parameter.h"
#include "macros.h"

namespace toy {
class ReprojectionCost;
class MapPointLinearization {
public:
  USING_SMART_PTR(MapPointLinearization);
  MapPointLinearization() = delete;
  MapPointLinearization(MapPointParameter*  rpMapPointParameter,
                        std::map<int, int>* frameIdColMap,
                        std::vector<std::shared_ptr<ReprojectionCost>>& costs);
  MapPointLinearization(MapPointLinearization&& src) noexcept;

  ~MapPointLinearization() = default;

  double linearize(bool updateState);
  void   decomposeWithQR();

  double backSubstitue(Eigen::VectorXd& frameDelta);

protected:
  MapPointParameter*                             mRpMapPointParameter;
  std::vector<std::shared_ptr<ReprojectionCost>> mReprojectionCosts;

  std::map<int, int>* mFrameIdColumnMapRp;
  Eigen::MatrixXd     mJ;
  Eigen::VectorXd     mC;
  int                 mRows;
  int                 mCols;

public:
  const Eigen::MatrixXd& J() const { return mJ; };
  Eigen::MatrixXd&       getJ() { return mJ; }

  const Eigen::VectorXd& C() const { return mC; };
};
}  //namespace toy