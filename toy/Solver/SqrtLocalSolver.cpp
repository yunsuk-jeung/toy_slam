#include <numeric>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#include "ToyLogger.h"
#include "config.h"
#include "Feature.h"
#include "ImagePyramid.h"
#include "Camera.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Factor.h"
#include "SqrtMarginalizationCost.h"
#include "CostFunction.h"
#include "SqrtMarginalizer.h"
#include "SqrtMarginalizationCost.h"
#include "MapPointLinearization.h"
#include "SqrtProblem.h"
#include "SqrtLocalSolver.h"

namespace toy {
namespace {
size_t MP_SIZE    = db::MapPoint::PARAMETER_SIZE;
size_t FRAME_SIZE = db::Frame::PARAMETER_SIZE;

MEstimator::Ptr createReprojectionMEstimator() {
  switch (Config::Vio::reprojectionME) {
  case 1: {
    return std::make_shared<Huber>(Config::Vio::reprojectionMEConst);
    break;
  }
  case 0:
  default:
    break;
  }
  return nullptr;
}
}  //namespace

SqrtLocalSolver::SqrtLocalSolver() {
  mProblem      = std::make_unique<SqrtProblem>();
  mMarginalizer = std::make_unique<SqrtMarginalizer>();

  const size_t& initialFrameSize = Config::Vio::solverMinimumFrames - 1;
  const auto    cols             = FRAME_SIZE * initialFrameSize;

  Eigen::MatrixXd& J   = mMarginalizer->getJ();
  Eigen::VectorXd& Res = mMarginalizer->getRes();

  J.resize(FRAME_SIZE, cols);
  Res.resize(FRAME_SIZE);

  J.setIdentity();
  J *= 10000.0;

  Res.setZero();
}

SqrtLocalSolver::~SqrtLocalSolver() {}

bool SqrtLocalSolver::solve(const std::vector<db::Frame::Ptr>&    frames,
                            const std::vector<db::MapPoint::Ptr>& trackingMapPoints,
                            const std::vector<db::MapPoint::Ptr>& marginedMapPoints) {
  if (frames.size() < Config::Vio::solverMinimumFrames)
    return false;

  mFrames    = &frames;
  mMapPoints = &trackingMapPoints;

  for (auto& f : *mFrames) {
    if (!f->isLinearized()) {
      f->resetDelta();
    }
  }

  /*problem->mOption.mLambda;*/
  mProblem->reset();
  mProblem->setFrames(mFrames);
  mProblem->setMapPoints(mMapPoints);

  //reprojection cost
  MEstimator::Ptr reProjME       = createReprojectionMEstimator();
  const double&   stdFocalLength = Config::Vio::standardFocalLength;

  //add margin mappoints for anchor
  //{
  //  using PORC = PoseOnlyReporjectinCost;
  //  std::vector<PORC::Ptr> costs;
  //  costs.reserve(marginedMapPoints.size() * frames.size());
  //  for (auto& mp : marginedMapPoints) {
  //    auto& frameFactors = mp->getFrameFactors();
  //    if (mp->status() == db::MapPoint::Status::MARGINED) {
  //      for (auto& frameFactor : frameFactors) {
  //        db::Frame::Ptr   frame0  = frameFactor.first.lock();
  //        Eigen::Vector3d& undist0 = frameFactor.second.undist0();
  //        Sophus::SE3d&    Tbc0    = frame0->getTbc(0);

  //PORC::Ptr cost = std::make_shared<PORC>(frame0, Tbc0, mp, undist0, reProjME);
  //costs.push_back(cost);

  //if (frameFactor.second.type() != db::ReprojectionFactor::Type::STEREO)
  //  continue;

  //Eigen::Vector3d& undist1 = frameFactor.second.undist1();
  //Sophus::SE3d&    Tbc1    = frame0->getTbc(1);
  //PORC::Ptr cost2 = std::make_shared<PORC>(frame0, Tbc1, mp, undist1, reProjME);
  //costs.push_back(cost2);
  //}
  //}
  //}
  //mProblem->addPoseOnlyReprojectionCost(costs);
  //}

  for (auto& mp : *mMapPoints) {
    auto& frameFactors = mp->frameFactorMap();

    std::vector<ReprojectionCost::Ptr> costs;
    costs.reserve(frameFactors.size());

    //auto it  = frameFactors.begin();
    //auto end = frameFactors.end();

    db::Frame::Ptr frame0 = mp->hostFrame();
    Sophus::SE3d&  Tbc0   = frame0->getTbc(0);

    for (auto& [frameCamId, factor] : frameFactors) {
      db::Frame::Ptr frame1 = factor.frame();
      auto&          camId  = frameCamId.camId;
      Sophus::SE3d&  Tbc1   = frame1->getTbc(camId);
      auto&          undist = factor.undist();

      ReprojectionCost::Ptr cost = std::make_shared<ReprojectionCost>(frame0,
                                                                      Tbc0,
                                                                      frame1,
                                                                      Tbc1,
                                                                      mp,
                                                                      undist,
                                                                      reProjME,
                                                                      stdFocalLength);
      costs.push_back(cost);
    }

    mProblem->addReprojectionCost(mp, costs);
  }

  mMarginalizer->setFrames(*mFrames);
  auto marginCost = mMarginalizer->createMarginCost();
  mProblem->addMarginalizationCost(marginCost);

  auto result = mProblem->solve();

  return result;
}

void SqrtLocalSolver::marginalize(db::Frame::Ptr marginalFrame) {
  std::vector<db::MapPoint::Ptr> marginMps;
  marginMps.reserve(marginalFrame->getFeature(0)->getKeypoints().size());

  const auto marginFrameId = marginalFrame->id();

  //greb mp which host frame is marginalFrame
  for (auto& mp : *mMapPoints) {
    auto frameId = mp->hostFrame()->id();
    if (frameId == marginFrameId) {
      marginMps.push_back(mp);
    }
  }

  if (Config::Vio::debug) {
    ToyLogD("MarginalFrame ID : {}, margin mapPoints : {} ",
            marginalFrame->id(),
            marginMps.size());
  }

  /*  get linearized and decomposed mappoint blocks  */

  size_t     rows = 0u;
  const auto cols = mFrames->size() * db::Frame::PARAMETER_SIZE;

  auto mpLinearizations = mProblem->grepMarginMapPointLinearizations(marginMps);

  if (Config::Vio::tbb) {
    auto sumRow = [&](const tbb::blocked_range<size_t>& r, size_t row) {
      for (size_t i = r.begin(); i != r.end(); ++i) {
        auto& linearization = mpLinearizations[i];
        linearization->linearize(true);
        linearization->decomposeWithQR();
        row += (linearization->J().rows() - MP_SIZE);
      }
      return row;
    };
    auto                       mpLinearizationSize = mpLinearizations.size();
    tbb::blocked_range<size_t> range(0, mpLinearizationSize);
    rows = tbb::parallel_reduce(range, size_t(0), sumRow, std::plus<size_t>());
  }
  else {
    for (auto& linearization : mpLinearizations) {
      linearization->linearize(true);
      linearization->decomposeWithQR();
      rows += (linearization->J().rows() - MP_SIZE);
    }
  }

  /*  imu factor */

  /* marginal factor*/
  rows += mMarginalizer->J().rows();

  /*  construct entire Jacob  */
  Eigen::MatrixXd Q2t_J;
  Eigen::VectorXd Q2t_C;

  Q2t_J.resize(rows, cols);
  Q2t_C.resize(rows);
  Q2t_J.setZero();
  Q2t_C.setZero();

  size_t currRow = 0;
  for (auto& linearization : mpLinearizations) {
    auto blockRow = linearization->J().rows() - MP_SIZE;
    // clang-format off
    auto& Q2t_J_block = 
    Q2t_J.block(currRow, 0, blockRow, cols) 
      = linearization->J().bottomLeftCorner(blockRow, cols);
    Q2t_C.segment(currRow, blockRow) = linearization->Res().tail(blockRow);
    // clang-format on
    currRow += blockRow;
  }

  {
    auto& J   = mMarginalizer->getJ();
    auto& Res = mMarginalizer->getRes();

    Q2t_J.block(currRow, 0, J.rows(), J.cols()) = J;
    Q2t_C.segment(currRow, J.rows())            = Res;
    currRow += J.rows();
  }

  std::map<int64_t, size_t>& frameColumnMap = mProblem->getFrameIdColumnMap();
  auto                       marginColStart = frameColumnMap[marginFrameId];

  Eigen::VectorXi indices(cols);
  auto            indexBegin = indices.begin();

  std::iota(indexBegin, indexBegin + FRAME_SIZE, marginColStart);
  indexBegin += FRAME_SIZE;

  for (auto& [id, startCol] : frameColumnMap) {
    if (id == marginFrameId)
      continue;
    std::iota(indexBegin, indexBegin + FRAME_SIZE, startCol);
    indexBegin += FRAME_SIZE;
  }

  Eigen::VectorXd delta(cols - db::Frame::PARAMETER_SIZE);
  Eigen::Index    deltaIdx = 0u;

  for (auto& f : *mFrames) {
    if (f->isKeyFrame()) {
      f->setLinearized(true);
    }
    if (f->id() == marginFrameId) {
      continue;
    }
    delta.segment(deltaIdx, FRAME_SIZE) = f->getDelta();
    deltaIdx += db::Frame::PARAMETER_SIZE;
  }
  //ToyLogD("wtf {}", delta);
  mMarginalizer->marginalize(indices, Q2t_J, Q2t_C, delta);
}

}  //namespace toy