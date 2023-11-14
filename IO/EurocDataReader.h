#pragma once
#include "DataReader.h"

namespace io {
class EurocReader : public DataReader {
public:
  void openDirectory(std::string dataDir) override;

protected:
};
}  //namespace io