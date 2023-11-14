#pragma once
#include <memory>
#include "Sensor.h"

namespace io {
class DataReader;
class Simulator : public Sensor {
public:
  Simulator();
  ~Simulator();

  void prepare() override;
  void getInfo(float* info) override;

  void start() override;
  void stop() override;

  void registerDataReader(DataReader* dataReader);

protected:
  DataReader* mDataReader;
};
}  //namespace io