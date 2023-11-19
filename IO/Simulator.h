#pragma once
#include <mutex>
#include <thread>
#include <atomic>
#include "Sensor.h"

class CameraInfo;
namespace io {
class DataReader;
class Simulator : public Sensor {
public:
  Simulator();
  ~Simulator();

  void prepare() override;
  void getInfo(CameraInfo* info0, CameraInfo* info1) override;

  void start() override;
  void stop() override;

  void spinOnce();

  void registerDataReader(DataReader* dataReader);

protected:
  DataReader* mDataReader;

  std::atomic<bool> mWorking = false;
  std::thread       mThread;

  int mImageType0;
  int mImageType1;
};
}  //namespace io