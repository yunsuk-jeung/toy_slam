#include "DataReader.h"
#include "Simulator.h"

namespace io {
Simulator::Simulator() {
  mIsSimulator = true;
}

Simulator::~Simulator() {
  delete mDataReader;
  mDataReader = nullptr;
}

void Simulator::prepare() {}

void Simulator::getInfo(float* info) {}

void Simulator::start() {}

void Simulator::stop() {}

void Simulator::registerDataReader(DataReader* dataReader) {
  mDataReader = dataReader;
}

}  //namespace io