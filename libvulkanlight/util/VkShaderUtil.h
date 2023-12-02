#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <set>
#include <unordered_map>

#include <glslang/Public/ShaderLang.h>
#include <volk.h>
#include <vulkan/vulkan.hpp>
namespace vkl {
class DirStackFileIncluder : public glslang::TShader::Includer {
public:
  DirStackFileIncluder()
    : externalLocalDirectoryCount(0) {}

  virtual IncludeResult* includeLocal(const char* headerName,
                                      const char* includerName,
                                      size_t      inclusionDepth) override {
    return readLocalPath(headerName, includerName, (int)inclusionDepth);
  }

  virtual IncludeResult* includeSystem(const char* headerName,
                                       const char* /*includerName*/,
                                       size_t /*inclusionDepth*/) override {
    return readSystemPath(headerName);
  }

  //Externally set directories. E.g., from a command-line -I<dir>.
  // - Most-recently pushed are checked first.
  // - All these are checked after the parse-time stack of local directories
  //   is checked.
  // - This only applies to the "local" form of #include.
  // - Makes its own copy of the path.
  virtual void pushExternalLocalDirectory(const std::string& dir) {
    directoryStack.push_back(dir);
    externalLocalDirectoryCount = (int)directoryStack.size();
  }

  virtual void releaseInclude(IncludeResult* result) override {
    if (result != nullptr) {
      delete[] static_cast<tUserDataElement*>(result->userData);
      delete result;
    }
  }

  virtual std::set<std::string> getIncludedFiles() { return includedFiles; }

  virtual ~DirStackFileIncluder() override {}

protected:
  typedef char             tUserDataElement;
  std::vector<std::string> directoryStack;
  int                      externalLocalDirectoryCount;
  std::set<std::string>    includedFiles;

  //Search for a valid "local" path based on combining the stack of include
  //directories and the nominal name of the header.
  virtual IncludeResult* readLocalPath(const char* headerName,
                                       const char* includerName,
                                       int         depth) {
    //Discard popped include directories, and
    //initialize when at parse-time first level.
    directoryStack.resize(depth + externalLocalDirectoryCount);
    if (depth == 1) directoryStack.back() = getDirectory(includerName);

    //Find a directory that works, using a reverse search of the include stack.
    for (auto it = directoryStack.rbegin(); it != directoryStack.rend(); ++it) {
      std::string path = *it + '/' + headerName;
      std::replace(path.begin(), path.end(), '\\', '/');
      std::ifstream file(path, std::ios_base::binary | std::ios_base::ate);
      if (file) {
        directoryStack.push_back(getDirectory(path));
        includedFiles.insert(path);
        return newIncludeResult(path, file, (int)file.tellg());
      }
    }

    return nullptr;
  }

  //Search for a valid <system> path.
  //Not implemented yet; returning nullptr signals failure to find.
  virtual IncludeResult* readSystemPath(const char* /*headerName*/) const {
    return nullptr;
  }

  //Do actual reading of the file, filling in a new include result.
  virtual IncludeResult* newIncludeResult(const std::string& path,
                                          std::ifstream&     file,
                                          int                length) const {
    char* content = new tUserDataElement[length];
    file.seekg(0, file.beg);
    file.read(content, length);
    return new IncludeResult(path, content, length, content);
  }

  //If no path markers, return current working directory.
  //Otherwise, strip file name and return path leading up to it.
  virtual std::string getDirectory(const std::string path) const {
    size_t last = path.find_last_of("/\\");
    return last == std::string::npos ? "." : path.substr(0, last);
  }
};

class ShaderVariant {
public:
  ShaderVariant() = default;

  ShaderVariant(std::string&& preamble, std::vector<std::string>&& processes);

  size_t get_id() const;

  /**
   * @brief Add definitions to shader variant
   * @param definitions Vector of definitions to add to the variant
   */
  void add_definitions(const std::vector<std::string>& definitions);

  /**
   * @brief Adds a define macro to the shader
   * @param def String which should go to the right of a define directive
   */
  void add_define(const std::string& def);

  /**
   * @brief Adds an undef macro to the shader
   * @param undef String which should go to the right of an undef directive
   */
  void add_undefine(const std::string& undef);

  /**
   * @brief Specifies the size of a named runtime array for automatic reflection. If
   * already specified, overrides the size.
   * @param runtime_array_name String under which the runtime array is named in the shader
   * @param size Integer specifying the wanted size of the runtime array (in number of
   * elements, not size in bytes), used for automatic allocation of buffers. See
   * get_declared_struct_size_runtime_array() in spirv_cross.h
   */
  void add_runtime_array_size(const std::string& runtime_array_name, size_t size);

  void set_runtime_array_sizes(const std::unordered_map<std::string, size_t>& sizes);

  const std::string& get_preamble() const;

  const std::vector<std::string>& get_processes() const;

  const std::unordered_map<std::string, size_t>& get_runtime_array_sizes() const;

  void clear();

private:
  size_t id;

  std::string preamble;

  std::vector<std::string> processes;

  std::unordered_map<std::string, size_t> runtime_array_sizes;

  void update_id();
};

///Helper class to generate SPIRV code from GLSL source
///A very simple version of the glslValidator application
class GLSLCompiler {
private:
  static glslang::EShTargetLanguage        env_target_language;
  static glslang::EShTargetLanguageVersion env_target_language_version;

public:
  /**
   * @brief Set the glslang target environment to translate to when generating code
   * @param target_language The language to translate to
   * @param target_language_version The version of the language to translate to
   */
  static void set_target_environment(
    glslang::EShTargetLanguage        target_language,
    glslang::EShTargetLanguageVersion target_language_version);

  /**
   * @brief Reset the glslang target environment to the default values
   */
  static void reset_target_environment();

  /**
   * @brief Compiles GLSL to SPIRV code
   * @param stage The Vulkan shader stage flag
   * @param glsl_source The GLSL source code to be compiled
   * @param entry_point The entrypoint function name of the shader stage
   * @param shader_variant The shader variant
   * @param[out] spirv The generated SPIRV code
   * @param[out] info_log Stores any log messages during the compilation process
   */

public:
  inline bool compile_to_spirv(vk::ShaderStageFlagBits     stage,
                               const std::vector<uint8_t>& glsl_source,
                               const std::string&          entry_point,
                               const ShaderVariant&        shader_variant,
                               std::vector<std::uint32_t>& spirv,
                               std::string&                info_log) {
    return compile_to_spirv(static_cast<VkShaderStageFlagBits>(stage),
                            glsl_source,
                            entry_point,
                            shader_variant,
                            spirv,
                            info_log);
  }

  bool compile_to_spirv(VkShaderStageFlagBits       stage,
                        const std::vector<uint8_t>& glsl_source,
                        const std::string&          entry_point,
                        const ShaderVariant&        shader_variant,
                        std::vector<std::uint32_t>& spirv,
                        std::string&                info_log);
};

class VkShaderUtil {
public:
  static vk::ShaderModule loadShader(vk::Device,
                                     uint32_t*              src,
                                     vk::DeviceSize         srcSize,
                                     std::vector<uint32_t>& spirv);

  static vk::ShaderModule loadShader(vk::Device             device,
                                     const std::string&     filename,
                                     std::vector<uint32_t>& spirv);

  static vk::ShaderModule loadShader(vk::Device              device,
                                     const std::string&      fileContents,
                                     vk::ShaderStageFlagBits stage,
                                     std::vector<uint32_t>&  spirv);

  static vk::ShaderModule compileShader(vk::Device                  device,
                                        vk::ShaderStageFlagBits     stage,
                                        const std::vector<uint8_t>& glsl_source,
                                        const std::string&          entry_point,
                                        const ShaderVariant&        shader_variant,
                                        std::vector<uint32_t>&      spirv);

protected:
  static std::vector<std::string> replaceInclude(const std::string& src);
  static std::string              readFileAsString(const std::string& fileName);
  static std::vector<uint8_t>     convertStringsToBytes(std::vector<std::string>& src);
};
}  //namespace vkl
