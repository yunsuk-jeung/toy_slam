#include "config.h"
#include "ToyAssert.h"
#include "CostFunction.h"
#include "DebugUtil.h"
#include "MapPointLinearization.h"

namespace toy {
namespace {
static constexpr auto COST_SIZE = ReprojectionCost::SIZE;
static constexpr auto POSE_SIZE = db::Frame::PARAMETER_SIZE;
static constexpr auto MP_SIZE   = db::MapPoint::PARAMETER_SIZE;
}  //namespace
MapPointLinearization::MapPointLinearization(db::MapPoint::Ptr          mp,
                                             std::map<int64_t, size_t>* frameIdColMap,
                                             std::vector<ReprojectionCost::Ptr>& costs)
  : mMapPoint{mp}
  , mFrameIdColumnMapRp{frameIdColMap} {
  mReprojectionCosts.swap(costs);

  int cols = mFrameIdColumnMapRp->size() * db::Frame::PARAMETER_SIZE;
  cols += db::MapPoint::PARAMETER_SIZE;

  auto costSize = mReprojectionCosts.size();
  mRows         = costSize << 1;  //uv
  mCols         = cols;

  mJ.resize(mRows, mCols);
  mJ.setZero();

  mRes.resize(mRows);
  mRes.setZero();
}

MapPointLinearization::MapPointLinearization(MapPointLinearization&& src) noexcept {
  mReprojectionCosts.swap(src.mReprojectionCosts);
}

double MapPointLinearization::linearize(bool updateState) {
  mJ.setZero();
  mRes.setZero();

  double errSq = 0;
  size_t row   = 0;

  const size_t mpCol = mJ.cols() - MP_SIZE;

  //YSTODO: tbb.... tbb might be slower
  for (auto& cost : mReprojectionCosts) {
    errSq += cost->linearlize(updateState);

    if (updateState) {
      const int& id0 = cost->getFrame0()->id();
      const int& id1 = cost->getFrame1()->id();

      TOY_ASSERT(mFrameIdColumnMapRp->find(id0) != mFrameIdColumnMapRp->end());
      TOY_ASSERT(mFrameIdColumnMapRp->find(id1) != mFrameIdColumnMapRp->end());

      const auto& idx0 = mFrameIdColumnMapRp->at(id0);
      const auto& idx1 = mFrameIdColumnMapRp->at(id1);

      mJ.block<COST_SIZE, POSE_SIZE>(row, idx0) = cost->J_f0();
      mJ.block<COST_SIZE, POSE_SIZE>(row, idx1) = cost->J_f1();
      mJ.block<COST_SIZE, MP_SIZE>(row, mpCol)  = cost->J_mp();
      mRes.segment(row, COST_SIZE)              = cost->Res();
      row += COST_SIZE;
    }
  }
  return errSq;
}

void MapPointLinearization::decomposeWithQR() {
  Eigen::VectorXd buffer0(mCols);
  Eigen::VectorXd buffer1(mRows - MP_SIZE);
  const auto      mpIdx = mCols - MP_SIZE;

  //ToyLogD("11111111111111111111111111");
  //ToyLogD("house holder test J : {}", ToyLogger::eigenMat(mJ));
  //ToyLogD("house holder test Res : {}", ToyLogger::eigenVec(mRes));

  for (size_t k = 0u; k < MP_SIZE; ++k) {
    size_t remainingRows = mRows - k;

    double beta;
    double tau;
    mJ.col(mpIdx + k).segment(k, remainingRows).makeHouseholder(buffer1, tau, beta);

    mJ.block(k, 0, remainingRows, mCols)
      .applyHouseholderOnTheLeft(buffer1, tau, buffer0.data());

    mRes.segment(k, remainingRows)
      .applyHouseholderOnTheLeft(buffer1, tau, buffer0.data());
    //ToyLogD("house holder test J : {}", ToyLogger::eigenMat(mJ));
    //ToyLogD("house holder test Res : {}", ToyLogger::eigenVec(mRes));
    //ToyLogD("");
  }
  //ToyLogD("2222222222222222222222222222");
  //ToyLogD("house holder test J : {}", ToyLogger::eigenMat(mJ));
  //ToyLogD("house holder test Res : {}", ToyLogger::eigenVec(mRes));
  //ToyLogD("");
}

double MapPointLinearization::backSubstitue(Eigen::VectorXd& frameDelta) {
  const auto mpColIdx = mCols - MP_SIZE;

  const auto Q1t_C  = mRes.head(MP_SIZE);
  const auto Q1t_Jp = mJ.topLeftCorner(MP_SIZE, mpColIdx);
  const auto Q1t_Jl = mJ.block<MP_SIZE, MP_SIZE>(0, mpColIdx)
                        .triangularView<Eigen::Upper>();

  Eigen::Vector<double, MP_SIZE> mpDelta = -Q1t_Jl.solve(Q1t_C + Q1t_Jp * frameDelta);
  mMapPoint->update(mpDelta);

  if (Config::Vio::compareLinearizedDiff) {
    TOY_ASSERT_MESSAGE(false, "not implemented");
    return 100.0;
  }

  return 0.0;
}
}  //namespace toy
