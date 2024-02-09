#pragma once
#include <string>
#include <map>

namespace vkl {
class ShaderPool {
public:
  static void                          init();
  static std::map<size_t, std::string> shaderMap;
  static const std::string&            requestShaderStr(const std::string& name);
};
}  //namespace vkl