#pragma once

#include <string>

#include "types.h"
#include "Singleton.h"

namespace toy {
class Vio;
class SLAM : public Singleton<SLAM> {
  friend class Singleton<SLAM>;

public:
  void prepare(const std::string& configFile);

  void setNewImage(const int       type,
                   const int       format,
                   const uint64_t& ns,
                   uint8_t*        buffer,
                   const int       l,
                   const int       w,
                   const int       h);

  void setAcc(const uint64_t& ns, float* acc);
  void setGyr(const uint64_t& ns, float* gyr);

private:
  SLAM();
  ~SLAM() override;

  Vio* vio;
};
}  //namespace toy