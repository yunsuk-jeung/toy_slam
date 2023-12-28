#include "ToyAssert.h"
#include "CostFunction.h"
#include "MapPointLinearization.h"

namespace toy {
MapPointLinearization::MapPointLinearization(
  std::map<int, FrameParameter>*      rpFrameParametersMap,
  std::vector<ReprojectionCost::Ptr>& costs)
  : mRpFrameParameterMap{rpFrameParametersMap} {
  mReprojectionCosts.swap(costs);

  int cols = 0;

  auto frameParamMapSize = mRpFrameParameterMap->size();
  for (auto it = mRpFrameParameterMap->begin(); it != mRpFrameParameterMap->end(); ++it) {
    const int& id      = it->second.id();
    mFrameIdColMap[id] = cols;
    cols += FrameParameter::SIZE;
  }

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

      TOY_ASSERT(mFrameIdColMap.find(id0) != mFrameIdColMap.end());
      TOY_ASSERT(mFrameIdColMap.find(id1) != mFrameIdColMap.end());

      auto idx0 = mFrameIdColMap[id0];
      auto idx1 = mFrameIdColMap[id1];

      auto& COST_SIZE = ReprojectionCost::SIZE;

      mJ.block<COST_SIZE, FrameParameter::SIZE>(row, idx0)     = cost->get_J_f0();
      mJ.block<COST_SIZE, FrameParameter::SIZE>(row, idx1)     = cost->get_J_f1();
      mJ.block<COST_SIZE, MapPointParameter::SIZE>(row, mpCol) = cost->get_J_mp();
      mC.segment(row, COST_SIZE)                               = cost->get_cost();
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
}
double MapPointLinearization::backSubstitue(Eigen::VectorXd& frameDelta) {
  
  return 0.0;
}
}  //namespace toy
