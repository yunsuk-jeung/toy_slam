#pragma once

namespace io {
class Sensor;
class SensorFactory {
public:
  enum SensorType {
    SIMULATOR,
  };

  static Sensor* createSensor(SensorType type);
};
}  //namespace io
