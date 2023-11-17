#pragma once
#include <string>
#include "types.h"

namespace toy {
class Config {
public:
  static void parseConfig(const std::string& file);
  static bool sync;

  struct Vio {
    static CamInfo camInfo0;
    static CamInfo camInfo1;
    static ImuInfo imuInfo;

    static std::string pointExtractor;
    static std::string pointMatcher;

    static int pyramidLevel;
    static int patchSize;

    static std::string lineExtractor;
    static std::string lineMatcher;

    static std::string solverType;
  };
};
}  //namespace toy