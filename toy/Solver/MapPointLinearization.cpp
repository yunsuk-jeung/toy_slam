#include "ToyLogger.h"
#include "CostFunction.h"
#include "MapPointLinearization.h"

namespace toy {
MapPointLinearization::MapPointLinearization(std::vector<ReprojectionCost::Ptr>& costs) {
  mReprojectionCosts.swap(costs);

  int cols = 0;

  for (auto& reprojectionCost : mReprojectionCosts) {
    auto*      fp0 = reprojectionCost->getFrameParameter0();
    const int& id0 = fp0->mId;
    if (mFrameIdColMap.find(id0) == mFrameIdColMap.end()) {
      mFrameIdColMap[id0] = cols;
      cols += FrameParameter::SIZE;
    }

    auto*      fp1 = reprojectionCost->getFrameParameter1();
    const int& id1 = fp1->mId;
    if (mFrameIdColMap.find(id1) == mFrameIdColMap.end()) {
      mFrameIdColMap[id1] = cols;
      cols += FrameParameter::SIZE;
    }
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
  int    row   = 0;

  int mpCol = mJ.cols() - MapPointParameter::SIZE;

  for (auto& cost : mReprojectionCosts) {
    errSq += cost->linearlize(updateState);

    if (updateState) {
      auto id0 = cost->getFrameParameter0()->mId;
      auto id1 = cost->getFrameParameter1()->mId;

      assert(mFrameIdColMap.find(id0) != mFrameIdColMap.end());
      assert(mFrameIdColMap.find(id1) != mFrameIdColMap.end());

      auto idx0 = mFrameIdColMap[id0];
      auto idx1 = mFrameIdColMap[id1];

      mJ.block<2, FrameParameter::SIZE>(row, idx0)     = cost->get_J_f0();
      mJ.block<2, FrameParameter::SIZE>(row, idx1)     = cost->get_J_f1();
      mJ.block<2, MapPointParameter::SIZE>(row, mpCol) = cost->get_J_mp();
      mC.segment(row, 2)                               = cost->get_cost();
      row += 2;
    }
  }
  return errSq;
}

void MapPointLinearization::decomposeWithQR() {
  Eigen::VectorXd buffer0(mCols);
  Eigen::VectorXd buffer1(mRows - MapPointParameter::SIZE);
  const int       mpIdx = mCols - MapPointParameter::SIZE;

  ToyLogD("house holder test J : {}", ToyLogger::eigenMat(mJ));
  ToyLogD("house holder test C : {}", ToyLogger::eigenVec(mC));

  for (int k = 0; k < MapPointParameter::SIZE; ++k) {
    int remainingRows = mRows - k;

    double beta;
    double tau;
    mJ.col(mpIdx + k).segment(k, remainingRows).makeHouseholder(buffer1, tau, beta);

    mJ.block(k, 0, remainingRows, mCols)
      .applyHouseholderOnTheLeft(buffer1, tau, buffer0.data());

    mC.segment(k, remainingRows).applyHouseholderOnTheLeft(buffer1, tau, buffer0.data());

    ToyLogD("house holder test J : {}", ToyLogger::eigenMat(mJ));
    ToyLogD("house holder test C : {}", ToyLogger::eigenVec(mC));
    ToyLogD("");
  }
}
}  //namespace toy
