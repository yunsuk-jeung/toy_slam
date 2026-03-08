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

  tbb::concurrent_queue<IPtr>& getInQueue() { return in_queue_; }
  void registerOutQueue(tbb::concurrent_queue<OPtr>* out) { out_queue_ = out; }
  void insert(IPtr in) { in_queue_.push(in); }

protected:
  IPtr getLatestInput() {
    IPtr out;
    while (in_queue_.try_pop(out)) {
      if (!in_queue_.empty())
        continue;
      return out;
    }
    return nullptr;
  }

protected:
  tbb::concurrent_queue<IPtr>  in_queue_;
  tbb::concurrent_queue<OPtr>* out_queue_;
};

}  //namespace toy