#pragma once
#include <memory>

namespace io {
class Sensor;
class SensorFactory {
public:
  enum SensorType {
    SIMULATOR,
  };

  static std::unique_ptr<Sensor> createSensor(SensorType type);
};
}  //namespace io
