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

    static bool        debug;
    static int         pyramidLevel;
    static int         patchSize;
    static int         rowGridCount;
    static int         colGridCount;
    static std::string pointTracker;
    static int         minTrackedPoint;
    static float       minTrackedRatio;
    static double      epipolarThreashold;
    static bool        showExtraction;
    static bool        showMonoTracking;
    static bool        showStereoTracking;

    static std::string lineTracker;
    static bool        frameTrackerSolvePose;

    static int   initializeMapPointCount;
    static int   minKeyFrameCount;
    static float newKeframeFeatureRatio;
    static int   maxFrameSize;
    static int   maxKeyFrameSize;

    static double      minParallaxSqNorm;
    static std::string solverType;
    static size_t      solverMinimumFrames;
    static int         reprojectionME;
    static double      reprojectionMEConst;
    static double      standardFocalLength;
    static int         maxIteration;
    static bool        compareLinearizedDiff;
  };

  struct Solver {
    static double basicMinDepth;
    static double basicMaxDepth;
  };
};
}  //namespace toy