#pragma once

#include <filesystem>
#include <system_error>

namespace io::util {
inline bool createDirectory(const std::filesystem::path& dirPath) {
  std::error_code ec;
  std::filesystem::create_directories(dirPath, ec);
  return !ec;
}
}  // namespace io::util
