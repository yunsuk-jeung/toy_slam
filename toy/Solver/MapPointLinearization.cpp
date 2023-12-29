#include "config.h"
#include "ToyAssert.h"
#include "CostFunction.h"
#include "MapPointLinearization.h"

namespace toy {
MapPointLinearization::MapPointLinearization(MapPointParameter*  rpMpP,
                                             std::map<int, int>* frameIdColMap,
                                             std::vector<ReprojectionCost::Ptr>& costs)
  : mRpMapPointParameter{rpMpP}
  , mFrameIdColumnMapRp{frameIdColMap} {
  mReprojectionCosts.swap(costs);

  int cols = mFrameIdColumnMapRp->size() * FrameParameter::SIZE;

  cols += MapPointParameter::SIZE;

  auto costSize = mReprojectionCosts.size();
  mRows         = costSize << 1;  //uv
  mCols         = cols;

  mJ.resize(mRows, mCols);
  mJ.setZero();

  mC.resize(mRows);
  mC.setZero();
}

MapPointLinearization::MapPointLinearization(MapPointLinearization&& src) noexcept {
  mReprojectionCosts.swap(src.mReprojectionCosts);
}

double MapPointLinearization::linearize(bool updateState) {
  double errSq = 0;
  size_t row   = 0;

  size_t mpCol = mJ.cols() - MapPointParameter::SIZE;
  //YSTODO: tbb
  for (auto& cost : mReprojectionCosts) {
    errSq += cost->linearlize(updateState);

    if (updateState) {
      const int& id0 = cost->getFrameParameter0()->id();
      const int& id1 = cost->getFrameParameter1()->id();

      TOY_ASSERT(mFrameIdColumnMapRp->find(id0) != mFrameIdColumnMapRp->end());
      TOY_ASSERT(mFrameIdColumnMapRp->find(id1) != mFrameIdColumnMapRp->end());

      const auto& idx0 = mFrameIdColumnMapRp->at(id0);
      const auto& idx1 = mFrameIdColumnMapRp->at(id1);

      constexpr auto COST_SIZE = ReprojectionCost::SIZE;

      mJ.block<COST_SIZE, FrameParameter::SIZE>(row, idx0)     = cost->J_f0();
      mJ.block<COST_SIZE, FrameParameter::SIZE>(row, idx1)     = cost->J_f1();
      mJ.block<COST_SIZE, MapPointParameter::SIZE>(row, mpCol) = cost->J_mp();
      mC.segment(row, COST_SIZE)                               = cost->C();
      row += COST_SIZE;
    }
  }
  return errSq;
}

void MapPointLinearization::decomposeWithQR() {
  Eigen::VectorXd buffer0(mCols);
  Eigen::VectorXd buffer1(mRows - MapPointParameter::SIZE);
  const auto      mpIdx = mCols - MapPointParameter::SIZE;

  //ToyLogD("house holder test J : {}", ToyLogger::eigenMat(mJ));
  //ToyLogD("house holder test C : {}", ToyLogger::eigenVec(mC));

  for (size_t k = 0u; k < MapPointParameter::SIZE; ++k) {
    size_t remainingRows = mRows - k;

    double beta;
    double tau;
    mJ.col(mpIdx + k).segment(k, remainingRows).makeHouseholder(buffer1, tau, beta);

    mJ.block(k, 0, remainingRows, mCols)
      .applyHouseholderOnTheLeft(buffer1, tau, buffer0.data());

    mC.segment(k, remainingRows).applyHouseholderOnTheLeft(buffer1, tau, buffer0.data());

    //ToyLogD("house holder test J : {}", ToyLogger::eigenMat(mJ));
    //ToyLogD("house holder test C : {}", ToyLogger::eigenVec(mC));
    //ToyLogD("");
  }
  //ToyLogD("house holder test J : {}", ToyLogger::eigenMat(mJ));
  //ToyLogD("house holder test C : {}", ToyLogger::eigenVec(mC));
  //ToyLogD("");
}

double MapPointLinearization::backSubstitue(Eigen::VectorXd& frameDelta) {
  constexpr size_t MP_SIZE  = MapPointParameter::SIZE;
  const auto       mpColIdx = mCols - MP_SIZE;

  const auto Q1t_C  = mC.head(MP_SIZE);
  const auto Q1t_Jp = mJ.topLeftCorner(MP_SIZE, mpColIdx);
  const auto Q1t_Jl = mJ.block<MP_SIZE, MP_SIZE>(0, mpColIdx)
                        .triangularView<Eigen::Upper>();

  Eigen::Vector<double, MP_SIZE> mpDelta = -Q1t_Jl.solve(Q1t_C + Q1t_Jp * frameDelta);
  mRpMapPointParameter->update(mpDelta);

  if (Config::Vio::compareLinearizedDiff) {
    TOY_ASSERT_MESSAGE(false, "not implemented");
    return 100.0;
  }

  return 0.0;
}
}  //namespace toy
