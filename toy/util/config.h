#pragma once
#include <string>
namespace toy {
class Config {
public:
  static void parseConfig(const std::string& file);

  static bool useDouble;
  static int  vioSolverType;
};
}  //namespace toy