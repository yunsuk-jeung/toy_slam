#pragma once
#include "DataReader.h"

namespace io {
class EurocReader : public DataReader {
public:
  EurocReader();
  ~EurocReader();

  void openDirectory(std::string configFile,
                     std::string dataDir,
                     bool        uploadMemory = false) override;

  void getInfos(CamInfo&, CamInfo&) override;

  bool getImages(int&      imageType0,
                 uint64_t& ns0,
                 cv::Mat&  image0,
                 int&      imageType1,
                 uint64_t& ns1,
                 cv::Mat&  image1) override;

protected:
  void syncStereo();

  void parseConfig(std::string configFile) override;
  void load() override;
  void loadAsync() override;
};
}  //namespace io