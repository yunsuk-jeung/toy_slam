#pragma once
#include <string>
namespace toy {
class Config {
public:
  static void parseConfig(const std::string& file);

  static bool useDouble;
  static int  vioSolverType;

  struct Vio {
    static std::string pointExtractor;
    static std::string pointMatcher;

    static std::string lineExtractor;
    static std::string lineMatcher;

    static std::string solverType;
  };
};
}  //namespace toy