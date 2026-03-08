#pragma once
#include <string>
#include "types.h"

namespace toy {
class Config {
public:
  static void parseConfig(const std::string& file);
  static bool sync;

  struct Vio {
    static CameraInfo  camInfo0;
    static CameraInfo  camInfo1;
    static ImuInfo     imuInfo;
    static bool        debug;
    static bool        tbb;
    static int         maxPyramidLevel;
    static int         patchSize;
    static int         rowGridCount;
    static int         colGridCount;
    static std::string pointTracker;
    static bool        equalizeHistogram;
    static int         minTrackedPoint;
    static float       minTrackedRatio;
    static double      epipolarThreashold;
    static int         stereoTrackingInterval;
    static bool        showExtraction;
    static bool        showMonoTracking;
    static bool        showStereoTracking;

    static std::string lineTracker;
    static bool        frameTrackerSolvePose;

    static int   initializeMapPointCount;
    static float minTriangulationBaselineSq;
    static float newKeyFrameFeatureRatio;
    static int   newKeyFrameAfter;
    static float margFeatureConnectionRatio;
    static int   minKeyFrameCount;
    static int   maxFrameSize;
    static int   maxKeyFrameSize;

    static double      minParallaxSqNorm;
    static std::string solverType;
    static bool        solverLogDebug;
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