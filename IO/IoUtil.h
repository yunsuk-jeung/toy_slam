#pragma once

#include <vector>
#include <filesystem>

namespace io {
namespace util {

inline std::vector<std::filesystem::path>
getFiles(const std::filesystem::path& dir, const std::filesystem::path& extension) {
  std::vector<std::filesystem::path> out;

  std::cout << dir << std::endl;
  std::cout << extension << std::endl;

  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    if (entry.is_regular_file()) {
      std::filesystem::path filePath = entry.path();
      
      if (filePath.extension() == extension || filePath.extension() == extension) {
        out.push_back(filePath);
      }
    }
  }
  
  return out;
}

}  //namespace util
}  //namespace io
