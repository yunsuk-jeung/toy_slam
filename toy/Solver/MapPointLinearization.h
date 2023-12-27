#pragma once
#include <memory>
#include <vector>
#include <map>
#include <Eigen/Dense>

namespace toy {
class ReprojectionCost;
class MapPointLinearization {
public:
  MapPointLinearization() = delete;
  MapPointLinearization(std::vector<std::shared_ptr<ReprojectionCost>>& costs);
  MapPointLinearization(MapPointLinearization&& src) noexcept;

  ~MapPointLinearization() = default;

  double linearize(bool updateState);
  void   decomposeWithQR();

protected:
  std::vector<std::shared_ptr<ReprojectionCost>> mReprojectionCosts;
  std::map<int, int>                             mFrameIdColMap;
  Eigen::MatrixXd                                mJ;
  Eigen::VectorXd                                mC;
  int                                            mRows;
  int                                            mCols;
};
}  //namespace toy