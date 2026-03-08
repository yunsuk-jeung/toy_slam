#pragma once
#include <string>
#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <opencv2/opencv.hpp>
#include "types.h"

namespace io {
class DataReader {
public:
  enum class Type {
    EUROC,
  };

  DataReader()
    : mUploadMemory{false}
    , mImage0Type{0}
    , mImage1Type{0}
    , mLoading{false} {}

  virtual ~DataReader() {
    stop();

    mImageInfos0.clear();
    mImageInfos1.clear();
  };

  static DataReader* createDataReader(DataReader::Type dataType);
  virtual void       openDirectory(std::string configFile,
                                   std::string dataDir,
                                   bool        uploadMemory = false) = 0;

  virtual void getInfos(CameraInfo&, CameraInfo&) = 0;

  virtual bool getImages(int&      imageType0,
                         uint64_t& ns0,
                         cv::Mat&  image0,
                         int&      imageType1,
                         uint64_t& ns1,
                         cv::Mat&  image1) = 0;

protected:
  virtual void parseConfig(std::string configFile) = 0;

  virtual void load()      = 0;
  virtual void loadAsync() = 0;
  virtual void stop() {
    mLoading = false;
    if (mLoadThread.joinable())
      mLoadThread.join();
  }

protected:
  bool mUploadMemory;

  int                                          mImage0Type;
  std::deque<std::pair<uint64_t, std::string>> mImageInfos0;

  int                                          mImage1Type;
  std::deque<std::pair<uint64_t, std::string>> mImageInfos1;

  std::deque<uint64_t> mImageNsDeque0;
  std::deque<cv::Mat>  mImageDeque0;
  std::deque<uint64_t> mImageNsDeque1;
  std::deque<cv::Mat>  mImageDeque1;

  CameraInfo mCamInfo0;
  CameraInfo mCamInfo1;
  ImuInfo    mImuInfo;

  std::mutex        mLoadLock;
  std::atomic<bool> mLoading;
  std::thread       mLoadThread;

public:
  CameraInfo& getCameraInfo0() { return mCamInfo0; }
  CameraInfo& getCameraInfo1() { return mCamInfo1; }
};
}  //namespace io