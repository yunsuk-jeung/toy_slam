#pragma once

struct ImageData {
  int      type   = -1;
  int      format = 0;
  uint64_t ns     = 0;
  uint8_t* buffer = nullptr;
  int      w      = 0;
  int      h      = 0;
};

enum ImageType { NONE = -1, MAIN = 0, SUB = 1, DEPTH = 2 };