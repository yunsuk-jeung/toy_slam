#pragma once
#include <string>
namespace toy {
class Config {
public:
  static void parseConfig(const std::string& file);
  static bool sync;

  struct Vio {
    static float camInfo0[12];
    static float camInfo1[12];
    static float imuInfo[4];

    static std::string pointExtractor;
    static std::string pointMatcher;

    static std::string lineExtractor;
    static std::string lineMatcher;

    static std::string solverType;
  };
};
}  //namespace toy