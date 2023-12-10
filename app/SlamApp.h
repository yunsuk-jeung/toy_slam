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
class SlamApp : public App {
public:
  SlamApp();
  ~SlamApp();

  bool prepare() override;
  void run() override;

  void createPipelines() override;
  void onRender() override;

  void buildCommandBuffer() override;
  void updateUniform(int idx) override;

protected:
  //std::unique_ptr<AxisRenderer> mAxisRenderer;
  //std::unique_ptr<PathRenderer> mPathRenderer;

  std::unique_ptr<PointCloudRenderer> mPointCloudRenderer;

  io::Sensor* mSensor;

public:
  void registerSensor(io::Sensor* sensor) { mSensor = sensor; }
};
}  //namespace vkl