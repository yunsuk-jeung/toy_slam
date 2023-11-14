#include "Simulator.h"
#include "SensorFactory.h"

namespace io {
std::unique_ptr<Sensor> SensorFactory::createSensor(SensorType type) {

  switch (type) {
  case SIMULATOR:
    return std::make_unique<Simulator>();
  default:
    return nullptr;
  }
}

}  //namespace io