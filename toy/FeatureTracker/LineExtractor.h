#pragma once
#include <string>
namespace toy {

class LineExtractor {
public:
  LineExtractor() = delete;
  LineExtractor(std::string) {}
  ~LineExtractor() {}
};
}  //namespace toy