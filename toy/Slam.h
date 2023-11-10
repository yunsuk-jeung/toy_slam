#pragma once

#include <memory>
#include <string>

#include "Singleton.h"
#include "types.h"

namespace toy {
class Vio;
class SLAM : public Singleton<SLAM> {
  friend class Singleton<SLAM>;

public:
  void init(const std::string& configFile);

  void setNewImage(ImageType type,
                   uint64_t& ns,
                   uint8_t*  buffer,
                   int       lenght,
                   int       width,
                   int       height);

  void setAcc(uint64_t& ns, float* acc);
  void setGyr(uint64_t& ns, float* gyr);

private:
  SLAM();
  ~SLAM() override;

  std::unique_ptr<Vio> vioUPtr;
};
}  //namespace toy