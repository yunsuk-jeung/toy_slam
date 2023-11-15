#include "DataReader.h"
#include "Simulator.h"

namespace io {
Simulator::Simulator() : mImageType0{0}, mImageType1{0} {
  mIsSimulator = true;
}

Simulator::~Simulator() {
  stop();
}

void Simulator::prepare() {
  mDataReader->getInfos(mCamInfo0, mCamInfo1);
}

void Simulator::getInfo(float* info0, float* info1) {
  int i      = 0;
  info0[i++] = mCamInfo0.type;
  info0[i++] = mCamInfo0.w;
  info0[i++] = mCamInfo0.h;
  info0[i++] = mCamInfo0.intrinsic[0];
  info0[i++] = mCamInfo0.intrinsic[1];
  info0[i++] = mCamInfo0.intrinsic[2];
  info0[i++] = mCamInfo0.intrinsic[3];
  info0[i++] = mCamInfo0.distortion[0];
  info0[i++] = mCamInfo0.distortion[1];
  info0[i++] = mCamInfo0.distortion[2];
  info0[i++] = mCamInfo0.distortion[3];
  info0[i++] = mCamInfo0.distortion[4];
  memcpy(&info0[i], &mCamInfo0.bMc, sizeof(float) * 16);

  i          = 0;
  info1[i++] = mCamInfo1.type;
  info1[i++] = mCamInfo1.w;
  info1[i++] = mCamInfo1.h;
  info1[i++] = mCamInfo1.intrinsic[0];
  info1[i++] = mCamInfo1.intrinsic[1];
  info1[i++] = mCamInfo1.intrinsic[2];
  info1[i++] = mCamInfo1.intrinsic[3];
  info1[i++] = mCamInfo1.distortion[0];
  info1[i++] = mCamInfo1.distortion[1];
  info1[i++] = mCamInfo1.distortion[2];
  info1[i++] = mCamInfo1.distortion[3];
  info1[i++] = mCamInfo1.distortion[4];
  memcpy(&info1[i], &mCamInfo1.bMc, sizeof(float) * 16);
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

    while (mWorking) {
      if (mDataReader->getImages(type0, ns0, image0, type1, ns1, image1)) {
        mImageCallBack(type0, image0.type(), ns0, image0.data, image0.cols, image0.rows);
        mImageCallBack(type1, image1.type(), ns1, image1.data, image1.cols, image1.rows);

        cv::imshow("111", image0);
        cv::waitKey(33);
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
  if (mThread.joinable()) mThread.join();
}

void Simulator::registerDataReader(DataReader* dataReader) {
  mDataReader = dataReader;
}

}  //namespace io