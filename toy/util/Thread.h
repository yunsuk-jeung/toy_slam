#pragma once
#include <memory>
#include <tbb/concurrent_queue.h>

namespace toy {
template <typename IN_, typename OUT_>
class Thread {
public:
  Thread() {}
  virtual ~Thread() {}

  using IPtr = std::shared_ptr<IN_>;
  using OPtr = std::shared_ptr<OUT_>;

  tbb::concurrent_queue<IPtr>& getInQueue() { return mInQueue; }
  void registerOutQueue(tbb::concurrent_queue<OPtr>* out) { mOutQueue = out; }
  void insert(IPtr in) { mInQueue.push(in); }

protected:
  IPtr getLatestInput() {
    IPtr out;
    while (mInQueue.try_pop(out)) {
      if (!mInQueue.empty())
        continue;
      return out;
    }
    return nullptr;
  }

protected:
  tbb::concurrent_queue<IPtr>  mInQueue;
  tbb::concurrent_queue<OPtr>* mOutQueue;
};

}  //namespace toy