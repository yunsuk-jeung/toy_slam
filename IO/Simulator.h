#pragma once
#include <mutex>
#include <thread>
#include <atomic>
#include "Sensor.h"

namespace io {
class DataReader;
class Simulator : public Sensor {
public:
  Simulator();
  ~Simulator();

  void prepare() override;
  void getInfo(float* info0, float* info1) override;

  void start() override;
  void stop() override;

  void registerDataReader(DataReader* dataReader);

protected:
  DataReader* mDataReader;

  std::atomic<bool> mWorking = false;
  std::thread       mThread;

  int mImageType0;
  int mImageType1;
};
}  //namespace io