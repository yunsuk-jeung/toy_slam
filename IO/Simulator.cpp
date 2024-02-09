#include "Logger.h"
#include "DataReader.h"
#include "Simulator.h"

namespace io {
Simulator::Simulator()
  : mDataReader{nullptr}
  , mImageType0{0}
  , mImageType1{0}
  , mContinuousMode{true}
  , mSendImage{false}
  , mWorking{false}
  , mSkip{0} {
  mIsSimulator = true;
}

Simulator::~Simulator() {
  stop();

  mCamInfo0;
}

void Simulator::prepare() {
  mDataReader->getInfos(mCamInfo0, mCamInfo1);
}

void Simulator::getInfo(CameraInfo* info0, CameraInfo* info1) {
  *info0 = mCamInfo0;
  *info1 = mCamInfo1;
}

void Simulator::setContinuosMode(bool sendImage) {
  mContinuousMode = sendImage;
}

void Simulator::changeContinousMode() {
  mContinuousMode = !mContinuousMode;
  mImageCallbackCv.notify_one();
}

void Simulator::sendImage() {
  if (mContinuousMode) {
    return;
  }

  std::unique_lock<std::mutex> ulock(mImageCallbackMutex);
  mSendImage = true;
  mImageCallbackCv.notify_one();
}

void Simulator::start() {
  mWorking = true;

  auto loopFunc = [&]() {
    int      type0;
    uint64_t ns0;
    cv::Mat  image0;

    int      type1;
    uint64_t ns1;
    cv::Mat  image1;
    int      skipCount = 0;

    while (mWorking) {
      if (mDataReader->getImages(type0, ns0, image0, type1, ns1, image1)) {
        if (skipCount++ < mSkip) {
          continue;
        }

        ImageData imageData0{type0,
                             image0.type(),
                             ns0,
                             image0.data,
                             image0.cols,
                             image0.rows};

        ImageData imageData1{type1,
                             image1.type(),
                             ns1,
                             image1.data,
                             image1.cols,
                             image1.rows};

        if (!mContinuousMode) {
          std::unique_lock<std::mutex> ulock(mImageCallbackMutex);
          mImageCallbackCv.wait(ulock, [&]() { return mSendImage || mContinuousMode; });
          mSendImage = false;
        }

        mImageCallBack(imageData0, imageData1);

        cv::imshow("input", image0);
        cv::waitKey(1);
      }
      else {
        //mWorking = false;
      }
    }
  };

  mThread = std::thread(loopFunc);
}

void Simulator::stop() {
  mWorking = false;
  sendImage();
  if (mThread.joinable())
    mThread.join();
}

void Simulator::spinOnce() {
  int      type0;
  uint64_t ns0;
  cv::Mat  image0;

  int      type1;
  uint64_t ns1;
  cv::Mat  image1;

  if (mDataReader->getImages(type0, ns0, image0, type1, ns1, image1)) {
    ImageData imageData0{type0,
                         image0.type(),
                         ns0,
                         image0.data,
                         image0.cols,
                         image0.rows};

    ImageData imageData1{type1,
                         image1.type(),
                         ns1,
                         image1.data,
                         image1.cols,
                         image1.rows};

    mImageCallBack(imageData0, imageData1);

    cv::imshow("input", image0);
    //cv::waitKey(33);
  }
  else {
    //mWorking = false;
  }
}

void Simulator::registerDataReader(DataReader* dataReader) {
  mDataReader = dataReader;
}

}  //namespace io