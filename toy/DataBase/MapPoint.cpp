#include "MapPoint.h"
#include "MemoryPointerPool.h"
namespace toy {
namespace db {
void MapPointPtr::release() {
  MemoryPointerPool::release<MapPoint>(mPointer);
}

}  //namespace db
}  //namespace toy
