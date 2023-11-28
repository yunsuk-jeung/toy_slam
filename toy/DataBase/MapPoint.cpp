#include "MapPoint.h"
namespace toy {
namespace db {
int MapPoint::globalId = 0;

MapPoint::MapPoint() {
  mId = globalId++;
}

}  //namespace db
}  //namespace toy
