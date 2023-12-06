#include "MapPoint.h"
#include "Frame.h"
#include "SqrtLocalSolver.h"

namespace toy {
SqrtLocalSolver::SqrtLocalSolver() {}
SqrtLocalSolver::~SqrtLocalSolver() {}

bool SqrtLocalSolver::solve(std::vector<db::Frame::Ptr>&    frames,
                            std::vector<db::MapPoint::Ptr>& mapPoints) {
  return true;
}

}  //namespace toy