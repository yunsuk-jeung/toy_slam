#pragma once
#include "App.h"

namespace io {
class Sensor;
}

namespace vkl {
class AxisRenderer;
class PathRenderer;
class PointCloudRenderer;
class SampleRenderer;
class SLAMApp : public App {
public:
  SLAMApp();
  ~SLAMApp();

  bool prepare() override;
  void run() override;

  void createRenderers() override;
  void onRender() override;

  void buildCommandBuffer() override;
  void updateUniform(int idx) override;

protected:
  void createPointRenderer();
  void createOriginRenderer();
  void createAxisRenderer();

  void updateSLAMData();

protected:
  std::unique_ptr<AxisRenderer>       mOriginAxisRenderer;
  std::unique_ptr<PointCloudRenderer> mPointCloudRenderer;

  std::unique_ptr<AxisRenderer> mAxisRenderer;
  //std::unique_ptr<PathRenderer> mPathRenderer;

  io::Sensor* mSensor;

  std::vector<Eigen::Matrix4f> mIs;
  std::vector<Eigen::Matrix4f> mMWcs;
  std::vector<float>           mLocalPointClouds;

public:
  void registerSensor(io::Sensor* sensor) { mSensor = sensor; }
};
}  //namespace vkl