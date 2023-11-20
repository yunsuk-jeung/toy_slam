#pragma once
#include <string>
#include "types.h"

namespace toy {
class Config {
public:
  static void parseConfig(const std::string& file);
  static bool sync;

  struct Vio {
    static CameraInfo camInfo0;
    static CameraInfo camInfo1;
    static ImuInfo    imuInfo;

    static int pyramidLevel;
    static int patchSize;
    static int rowGridCount;
    static int colGridCount;

    static std::string pointTracker;
    static std::string lineTracker;

    static std::string solverType;

    static int mapFrameSize;
  };
};
}  //namespace toy