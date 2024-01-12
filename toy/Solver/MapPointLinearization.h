#pragma once
#include <memory>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include "macros.h"

namespace toy {
namespace db {
class MapPoint;
}
class ReprojectionCost;
class PoseOnlyReporjectinCost;
class MapPointLinearization {
public:
  USING_SMART_PTR(MapPointLinearization);
  MapPointLinearization() = delete;
  MapPointLinearization(std::shared_ptr<db::MapPoint>                   mp,
                        std::map<size_t, size_t>*                       frameIdColMap,
                        std::vector<std::shared_ptr<ReprojectionCost>>& costs);
  MapPointLinearization(MapPointLinearization&& src) noexcept;

  ~MapPointLinearization() = default;

  virtual double linearize(bool updateState);
  virtual void   decomposeWithQR();

  virtual double backSubstitue(Eigen::VectorXd& frameDelta);

protected:
  std::shared_ptr<db::MapPoint>                  mMapPoint;
  std::vector<std::shared_ptr<ReprojectionCost>> mReprojectionCosts;

  std::map<size_t, size_t>* mFrameIdColumnMapRp;
  Eigen::MatrixXd           mJ;
  Eigen::VectorXd           mRes;
  int                       mRows;
  int                       mCols;

public:
  const std::shared_ptr<db::MapPoint>& mp() const { return mMapPoint; }
  const Eigen::MatrixXd&               J() const { return mJ; };
  Eigen::MatrixXd&                     getJ() { return mJ; }

  const Eigen::VectorXd& Res() const { return mRes; };
};
}  //namespace toy