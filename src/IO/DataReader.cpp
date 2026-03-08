#include "EurocReader.h"
namespace io {
DataReader* DataReader::createDataReader(DataReader::Type dataType) {
  switch (dataType) {
  case DataReader::Type::EUROC:
    return new EurocReader();
    break;

  default:
    break;
  }
  return nullptr;
}
}  //namespace io