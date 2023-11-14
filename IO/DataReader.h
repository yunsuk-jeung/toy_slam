#pragma once
#include <string>

namespace io {
class DataReader {
public:
  enum class Type {
    EUROC,
  };

  DataReader()  = default;
  ~DataReader() = default;

  virtual void       openDirectory(std::string dataDir) = 0;
  static DataReader* createDataReader(Type dataType);

  //virtual void       getImage()      = 0;

protected:
};
}  //namespace io