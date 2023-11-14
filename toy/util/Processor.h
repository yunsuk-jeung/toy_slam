#pragma once

#include <tbb/concurrent_queue.h>

namespace toy {

template <class IN, class OUT>
class Processor {
public:
  Processor() {}
  virtual ~Processor() {}
  void registerOutQueue(tbb::concurrent_queue<OUT*>* out) { outQueue = out; }

  void insert(IN* in) { mInQueue.push(in); }

protected:
  IN* getLatestInput() {
    IN* out;
    while (mInQueue.try_pop(out)) {
      if (!mInQueue.empty())
        delete out;
      else
        return out;
    }
    return nullptr;
  }

protected:
  tbb::concurrent_queue<IN*>   mInQueue;
  tbb::concurrent_queue<OUT*>* mOutQueue;
};

}  //namespace toy