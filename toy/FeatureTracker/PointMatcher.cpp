#include "CVOpticalFlow.h"
#include "PatchOpticalFlow.h"
#include "PointMatcher.h"

namespace toy {
PointMatcher::Ptr PointMatcherFactory::create(const std::string& type) {
  if (type == "CVOpticalFlow") {
    return std::make_shared<CVOpticalFlow>();
  }
  else if (type == "PatchOpticalFlow") {
    return std::make_shared<PatchOpticalFlow>();
  }
  return nullptr;
}
}  //namespace toy