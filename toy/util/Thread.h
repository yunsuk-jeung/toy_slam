#pragma once

#include <tbb/concurrent_queue.h>
#include "MemoryPointerPool.h"

namespace toy {
template <typename IN_, typename OUT_>
class Thread {
public:
  Thread() {}
  virtual ~Thread() {}

  tbb::concurrent_queue<IN_*>& getInQueue() { return mInQueue; }
  void registerOutQueue(tbb::concurrent_queue<OUT_*>* out) { mOutQueue = out; }
  void insert(IN_* in) { mInQueue.push(in); }

protected:
  IN_* getLatestInput() {
    IN_* out;
    while (mInQueue.try_pop(out)) {
      if (!mInQueue.empty()) { db::MemoryPointerPool::release<IN_>(out); }
      else { return out; }
    }
    return nullptr;
  }

protected:
  tbb::concurrent_queue<IN_*>   mInQueue;
  tbb::concurrent_queue<OUT_*>* mOutQueue;
};

}  //namespace toy