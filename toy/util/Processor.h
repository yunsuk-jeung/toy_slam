#pragma once

#include <tbb/concurrent_queue.h>

namespace toy {

template <class IN, class OUT>
class Processor {
public:

  Processor() {}
  virtual ~Processor() {}

private:

  tbb::concurrent_queue<IN*>  inQueue;
  tbb::concurrent_queue<OUT*> outQueue;
};

}  //namespace toy