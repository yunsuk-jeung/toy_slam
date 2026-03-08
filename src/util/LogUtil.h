#pragma once

#include <cstring>

namespace LogUtil {
inline const char* extractFileName(const char* filePath) {
  if (!filePath) {
    return "";
  }

  const char* slash     = std::strrchr(filePath, '/');
  const char* backslash = std::strrchr(filePath, '\\');
  const char* pos       = (slash && backslash) ? (slash > backslash ? slash : backslash)
                                                : (slash ? slash : backslash);
  return pos ? pos + 1 : filePath;
}
}  // namespace LogUtil
