#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace toy {
template <typename FLOAT>
class Frame {
  using ptr = std::shared_ptr<Frame>;

public:
  
protected:

  Frame();
  ~Frame();

protected:

  Eigen::Vector<FLOAT, 3> wTc;
};
}  //namespace toy