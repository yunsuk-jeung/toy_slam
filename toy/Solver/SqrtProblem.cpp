#include "SqrtProblem.h"

namespace toy {
SqrtProblem::Option::Option()
  : mMaxIteration{7}
  , mLambda{1e-4}
  , mMaxLambda{1e2}
  , mMinLambda{1e-5}
  , mMu{2.0} {}

}  //namespace toy
