#pragma once

#include "util/Singleton.h"

namespace toy {
class SLAM : public Singleton<SLAM>{
  friend class Singleton<SLAM>;

public:

private:

  SLAM();
  ~SLAM() override;
};
}  //namespace toy