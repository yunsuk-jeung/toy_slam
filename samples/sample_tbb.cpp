#include <iostream>
#include <vector>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>

template <typename Scalar_>
struct Sample {
  Sample(std::vector<std::vector<Scalar_>>& vecs_)
    : vecs{vecs_}
    , sum{0} {}

  Sample(Sample& src, tbb::split)
    : sum{0}
    , vecs{src.vecs} {}

  void operator()(const tbb::blocked_range<size_t>& r) {
      for (size_t i = r.begin(); i != r.end(); ++i) {
        auto& vec = vecs[i];
        for (auto& v : vec) {
          sum += v;
        }
      }
  }

  inline void join(Sample& b) { sum += b.sum; }

  Scalar_                            sum;
  std::vector<std::vector<Scalar_>>& vecs;
};

int main() {
  using Scalar = int;

  std::vector<std::vector<Scalar>> vecs(10);

  for (int i = 0; i < 10; ++i) {
    vecs.push_back(std::vector<Scalar>());
    for (int j = 0; j < 10; ++j) {
      vecs[i].push_back((Scalar)j);
    }
  }

  {
    auto horizontalSum = [&](const tbb::blocked_range<size_t>& r, Scalar sum) {
      for (size_t i = r.begin(); i != r.end(); ++i) {
        auto& vec = vecs[i];
        for (auto& v : vec) {
          sum += v;
        }
      }
      return sum;
    };

    auto                       vecsSize = vecs.size();
    tbb::blocked_range<size_t> range(0, vecsSize);

    auto sum = tbb::parallel_reduce(range, Scalar(0), horizontalSum, std::plus<Scalar>());

    std::cout << "by lambda function : " << sum << std::endl;
  }

  {
    Sample<Scalar>             a{vecs};
    auto                       vecsSize = vecs.size();
    tbb::blocked_range<size_t> range(0, vecsSize);
    tbb::parallel_reduce(range, a);

    std::cout << "by struct : " << a.sum << std::endl;
  }

  return 0;
}