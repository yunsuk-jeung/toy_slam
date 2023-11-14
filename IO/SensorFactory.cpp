#include "Simulator.h"
#include "SensorFactory.h"

namespace io {
Sensor* SensorFactory::createSensor(SensorType type) {

  switch (type) {
  case SIMULATOR:
    return new Simulator();
  default:
    return nullptr;
  }
}

}  //namespace io