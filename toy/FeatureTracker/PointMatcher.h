#pragma once
#include <memory>
#include <string>
#include "macros.h"
namespace toy {
namespace db {
class Frame;
}  //namespace db
class PointMatcher {
public:
  USING_SMART_PTR(PointMatcher);
  PointMatcher()          = default;
  virtual ~PointMatcher() = default;

  virtual size_t match(db::Frame* prev, db::Frame* curr) = 0;
  virtual size_t matchStereo(db::Frame* frame)           = 0;

protected:
};

class PointMatcherFactory {
public:
  static PointMatcher::Ptr create(const std::string& type);
};
}  //namespace toy