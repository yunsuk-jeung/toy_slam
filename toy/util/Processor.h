#pragma once

#include <tbb/concurrent_queue.h>

namespace toy {

template <typename IN_, typename OUT_>
class Processor {
public:
  Processor() {}
  virtual ~Processor() {}
  void registerOutQueue(tbb::concurrent_queue<OUT_*>* out) { outQueue = out; }

  void insert(IN_* in) { mInQueue.push(in); }

protected:
  IN_* getLatestInput() {
    IN_* out;
    while (mInQueue.try_pop(out)) {
      if (!mInQueue.empty())
        delete[] out;
      else
        return out;
    }
    return nullptr;
  }

protected:
  tbb::concurrent_queue<IN_*>   mInQueue;
  tbb::concurrent_queue<OUT_*>* mOutQueue;
};

}  //namespace toy