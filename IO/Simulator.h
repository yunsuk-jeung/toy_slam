#pragma once
#include <mutex>
#include <condition_variable>
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

  void setContinuosMode(bool sendImage);
  void changeContinousMode();
  void sendImage();
  void start() override;
  void stop() override;

  void spinOnce();

  void registerDataReader(DataReader* dataReader);

protected:
  DataReader* mDataReader;

  bool                    mContinuousMode;
  bool                    mSendImage;
  std::condition_variable mImageCallbackCv;
  std::mutex              mImageCallbackMutex;

  std::atomic<bool> mWorking;
  std::thread       mThread;

  int mImageType0;
  int mImageType1;

public:
  bool& getContinuousMode() { return mContinuousMode; }
};
}  //namespace io