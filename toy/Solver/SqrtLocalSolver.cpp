#include "ToyLogger.h"
#include "config.h"
#include "Feature.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Factor.h"
#include "SqrtMarginalizer.h"
#include "CostFunction.h"
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

  Eigen::MatrixXd& J = mMarginalizer->getJ();
  Eigen::VectorXd& C = mMarginalizer->getC();

  J.resize(FRAME_SIZE, FRAME_SIZE);
  C.resize(FRAME_SIZE);

  J.setIdentity();
  J *= 100.0;

  C.setZero();
}

SqrtLocalSolver::~SqrtLocalSolver() {}

bool SqrtLocalSolver::solve(std::vector<db::Frame::Ptr>&    frames,
                            std::vector<db::MapPoint::Ptr>& mapPoints) {
  if (frames.size() < 4)
    return false;

  mFrames    = &frames;
  mMapPoints = &mapPoints;

  for (auto& f : *mFrames) {
    f->resetDelta();
  }

  /*problem->mOption.mLambda;*/
  mProblem->reset();
  mProblem->setFrames(mFrames);
  mProblem->setMapPoints(mMapPoints);

  MEstimator::Ptr reProjME    = createReprojectionMEstimator();
  const double&   focalLength = Config::Vio::standardFocalLength;

  for (auto& mp : mapPoints) {
    auto& frameFactors = mp->getFrameFactors();

    std::vector<ReprojectionCost::Ptr> costs;
    costs.reserve(frameFactors.size());

    auto it  = frameFactors.begin();
    auto end = frameFactors.end();

    db::Frame::Ptr frame0 = (*it).first.lock();
    Sophus::SE3d&  Tbc0   = frame0->getTbc(0);

    //add prior for uv
    {
      Eigen::Vector3d undist0 = it->second.undist0();

      ReprojectionCost::Ptr cost = std::make_shared<ReprojectionPriorCost>(frame0,
                                                                           Tbc0,
                                                                           frame0,
                                                                           Tbc0,
                                                                           mp,
                                                                           undist0,
                                                                           reProjME,
                                                                           focalLength);
      costs.push_back(cost);
    }
    //add stereo reprojection cost for sub cam
    if (it->second.getType() == db::ReprojectionFactor::Type::STEREO) {
      Sophus::SE3d&    Tbc1    = frame0->getTbc(1);
      Eigen::Vector3d& undist1 = it->second.undist1();

      ReprojectionCost::Ptr cost = std::make_shared<StereoReprojectionCost>(frame0,
                                                                            Tbc0,
                                                                            frame0,
                                                                            Tbc1,
                                                                            mp,
                                                                            undist1,
                                                                            reProjME,
                                                                            focalLength);
      costs.push_back(cost);
    }

    //add other reporjection cost
    ++it;
    for (; it != end; ++it) {
      db::Frame::Ptr   frame1  = (*it).first.lock();
      Sophus::SE3d&    Tbc1    = frame1->getTbc(0);
      Eigen::Vector3d& undist0 = it->second.undist0();

      ReprojectionCost::Ptr cost = std::make_shared<ReprojectionCost>(frame0,
                                                                      Tbc0,
                                                                      frame1,
                                                                      Tbc1,
                                                                      mp,
                                                                      undist0,
                                                                      reProjME,
                                                                      focalLength);
      costs.push_back(cost);

      if (it->second.getType() != db::ReprojectionFactor::Type::STEREO)
        continue;

      Sophus::SE3d&    Tbc2    = frame1->getTbc(1);
      Eigen::Vector3d& undist1 = it->second.undist1();

      ReprojectionCost::Ptr cost2 = std::make_shared<ReprojectionCost>(frame0,
                                                                       Tbc0,
                                                                       frame1,
                                                                       Tbc2,
                                                                       mp,
                                                                       undist1,
                                                                       reProjME,
                                                                       focalLength);
      costs.push_back(cost2);
    }

    mProblem->addReprojectionCost(mp, costs);
  }

  auto result = mProblem->solve();

  return result;
}

void SqrtLocalSolver::marginalize(db::Frame::Ptr marginalFrame) {
  std::vector<db::MapPoint::Ptr> marginMps;
  marginMps.reserve(marginalFrame->getFeature(0)->getKeypoints().size());

  //greb mp which host frame is marginalFrame
  for (auto& mp : *mMapPoints) {
    auto& frameFactor = mp->getFrameFactors();
    if (frameFactor.front().first.lock() == marginalFrame) {
      marginMps.push_back(mp);
    }
  }

  if (Config::Vio::debug) {
    ToyLogD("MarginalFrame ID : {}, margin mapPoints : {} ", marginMps.size());
  }
  //get linearized and decomposed mappoint blocks
  //YSTODO: TBB

  size_t     rows = 0u;
  const auto cols = mFrames->size() * db::Frame::PARAMETER_SIZE;

  auto mpLinearizations = mProblem->getMaPointLinearizations(marginMps);

  for (auto& linearization : mpLinearizations) {
    linearization->linearize(true);
    linearization->decomposeWithQR();
    rows += (linearization->J().rows() - MP_SIZE);
  }

  //marginal factor

  //imu factor

  //construct entire Jacob
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
    Q2t_J.block(currRow, 0, blockRow, cols) = linearization->J().bottomLeftCorner(blockRow, cols);
    Q2t_C.segment(currRow, blockRow) = linearization->C().tail(blockRow);
    // clang-format on
    currRow += blockRow;
    ToyLogD("testing marginalize {}", ToyLogger::eigenMat(Q2t_J));
  }
}

}  //namespace toy