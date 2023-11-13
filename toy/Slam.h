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
  void prepare(const std::string& configFile);

  void setNewImage(ImageType   type,
                   ImageFormat format,
                   uint64_t&   ns,
                   uint8_t*    buffer,
                   int         l,
                   int         w,
                   int         h);

  void setAcc(uint64_t& ns, float* acc);
  void setGyr(uint64_t& ns, float* gyr);

private:
  SLAM();
  ~SLAM() override;

  std::unique_ptr<Vio> vioUPtr;
};
}  //namespace toy