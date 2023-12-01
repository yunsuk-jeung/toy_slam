#include <SPIRV/GlslangToSpv.h>
#include <glslang/Public/ResourceLimits.h>

#include "VkShaderUtil.h"
#include "VkLogger.h"
#include "VkError.h"
#include "ResourcePool.h"

namespace vkl {

ShaderVariant::ShaderVariant(std::string&& preamble, std::vector<std::string>&& processes)
  : preamble{std::move(preamble)}
  , processes{std::move(processes)} {
  update_id();
}

size_t ShaderVariant::get_id() const {
  return id;
}

void ShaderVariant::add_definitions(const std::vector<std::string>& definitions) {
  for (auto& definition : definitions) { add_define(definition); }
}

void ShaderVariant::add_define(const std::string& def) {
  processes.push_back("D" + def);

  std::string tmp_def = def;

  //The "=" needs to turn into a space
  size_t pos_equal = tmp_def.find_first_of("=");
  if (pos_equal != std::string::npos) { tmp_def[pos_equal] = ' '; }

  preamble.append("#define " + tmp_def + "\n");

  update_id();
}

void ShaderVariant::add_undefine(const std::string& undef) {
  processes.push_back("U" + undef);

  preamble.append("#undef " + undef + "\n");

  update_id();
}

void ShaderVariant::add_runtime_array_size(const std::string& runtime_array_name,
                                           size_t             size) {
  if (runtime_array_sizes.find(runtime_array_name) == runtime_array_sizes.end()) {
    runtime_array_sizes.insert({runtime_array_name, size});
  }
  else { runtime_array_sizes[runtime_array_name] = size; }
}

void ShaderVariant::set_runtime_array_sizes(
  const std::unordered_map<std::string, size_t>& sizes) {
  this->runtime_array_sizes = sizes;
}

const std::string& ShaderVariant::get_preamble() const {
  return preamble;
}

const std::vector<std::string>& ShaderVariant::get_processes() const {
  return processes;
}

const std::unordered_map<std::string, size_t>& ShaderVariant::get_runtime_array_sizes()
  const {
  return runtime_array_sizes;
}

void ShaderVariant::clear() {
  preamble.clear();
  processes.clear();
  runtime_array_sizes.clear();
  update_id();
}

void ShaderVariant::update_id() {
  std::hash<std::string> hasher{};
  id = hasher(preamble);
}
namespace {
inline EShLanguage FindShaderLanguage(VkShaderStageFlagBits stage) {
  switch (stage) {
  case VK_SHADER_STAGE_VERTEX_BIT:
    return EShLangVertex;

  case VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT:
    return EShLangTessControl;

  case VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT:
    return EShLangTessEvaluation;

  case VK_SHADER_STAGE_GEOMETRY_BIT:
    return EShLangGeometry;

  case VK_SHADER_STAGE_FRAGMENT_BIT:
    return EShLangFragment;

  case VK_SHADER_STAGE_COMPUTE_BIT:
    return EShLangCompute;

  case VK_SHADER_STAGE_RAYGEN_BIT_KHR:
    return EShLangRayGen;

  case VK_SHADER_STAGE_ANY_HIT_BIT_KHR:
    return EShLangAnyHit;

  case VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR:
    return EShLangClosestHit;

  case VK_SHADER_STAGE_MISS_BIT_KHR:
    return EShLangMiss;

  case VK_SHADER_STAGE_INTERSECTION_BIT_KHR:
    return EShLangIntersect;

  case VK_SHADER_STAGE_CALLABLE_BIT_KHR:
    return EShLangCallable;

  case VK_SHADER_STAGE_MESH_BIT_EXT:
    return EShLangMesh;

  case VK_SHADER_STAGE_TASK_BIT_EXT:
    return EShLangTask;

  default:
    return EShLangVertex;
  }
}
}  //namespace

glslang::EShTargetLanguage
  GLSLCompiler::env_target_language = glslang::EShTargetLanguage::EShTargetNone;
glslang::EShTargetLanguageVersion GLSLCompiler::env_target_language_version = static_cast<
  glslang::EShTargetLanguageVersion>(0);

void GLSLCompiler::set_target_environment(
  glslang::EShTargetLanguage        target_language,
  glslang::EShTargetLanguageVersion target_language_version) {
  GLSLCompiler::env_target_language         = target_language;
  GLSLCompiler::env_target_language_version = target_language_version;
}

void GLSLCompiler::reset_target_environment() {
  GLSLCompiler::env_target_language         = glslang::EShTargetLanguage::EShTargetNone;
  GLSLCompiler::env_target_language_version = static_cast<
    glslang::EShTargetLanguageVersion>(0);
}

bool GLSLCompiler::compile_to_spirv(VkShaderStageFlagBits       stage,
                                    const std::vector<uint8_t>& glsl_source,
                                    const std::string&          entry_point,
                                    const ShaderVariant&        shader_variant,
                                    std::vector<std::uint32_t>& spirv,
                                    std::string&                info_log) {
  //Initialize glslang library.
  glslang::InitializeProcess();

  EShMessages messages = static_cast<EShMessages>(EShMsgDefault | EShMsgVulkanRules
                                                  | EShMsgSpvRules);

  EShLanguage language = FindShaderLanguage(stage);
  std::string source   = std::string(glsl_source.begin(), glsl_source.end());

  const char* file_name_list[1] = {""};
  const char* shader_source     = reinterpret_cast<const char*>(source.data());

  glslang::TShader shader(language);
  shader.setStringsWithLengthsAndNames(&shader_source, nullptr, file_name_list, 1);
  shader.setEntryPoint(entry_point.c_str());
  shader.setSourceEntryPoint(entry_point.c_str());
  shader.setPreamble(shader_variant.get_preamble().c_str());
  shader.addProcesses(shader_variant.get_processes());
  if (GLSLCompiler::env_target_language != glslang::EShTargetLanguage::EShTargetNone) {
    shader.setEnvTarget(GLSLCompiler::env_target_language,
                        GLSLCompiler::env_target_language_version);
  }

  DirStackFileIncluder includeDir;
  includeDir.pushExternalLocalDirectory("shaders");

  if (!shader.parse(GetDefaultResources(), 100, false, messages, includeDir)) {
    info_log = std::string(shader.getInfoLog()) + "\n"
               + std::string(shader.getInfoDebugLog());
    return false;
  }

  //Add shader to new program object.
  glslang::TProgram program;
  program.addShader(&shader);

  //Link program.
  if (!program.link(messages)) {
    info_log = std::string(program.getInfoLog()) + "\n"
               + std::string(program.getInfoDebugLog());
    return false;
  }

  //Save any info log that was generated.
  if (shader.getInfoLog()) {
    info_log += std::string(shader.getInfoLog()) + "\n"
                + std::string(shader.getInfoDebugLog()) + "\n";
  }

  if (program.getInfoLog()) {
    info_log += std::string(program.getInfoLog()) + "\n"
                + std::string(program.getInfoDebugLog());
  }

  glslang::TIntermediate* intermediate = program.getIntermediate(language);

  //Translate to SPIRV.
  if (!intermediate) {
    info_log += "Failed to get shared intermediate code.\n";
    return false;
  }

  spv::SpvBuildLogger logger;

  glslang::GlslangToSpv(*intermediate, spirv, &logger);

  info_log += logger.getAllMessages() + "\n";

  //Shutdown glslang library.
  glslang::FinalizeProcess();

  return true;
}
namespace {
vk::ShaderStageFlagBits find_shader_stage(const std::string& ext) {
  if (ext == "vert") { return vk::ShaderStageFlagBits::eVertex; }
  else if (ext == "frag") { return vk::ShaderStageFlagBits::eFragment; }
  else if (ext == "comp") { return vk::ShaderStageFlagBits::eCompute; }
  else if (ext == "geom") { return vk::ShaderStageFlagBits::eGeometry; }
  else if (ext == "tesc") { return vk::ShaderStageFlagBits::eTessellationControl; }
  else if (ext == "tese") { return vk::ShaderStageFlagBits::eTessellationEvaluation; }
  else if (ext == "rgen") { return vk::ShaderStageFlagBits::eRaygenKHR; }
  else if (ext == "rahit") { return vk::ShaderStageFlagBits::eAnyHitKHR; }
  else if (ext == "rchit") { return vk::ShaderStageFlagBits::eClosestHitKHR; }
  else if (ext == "rmiss") { return vk::ShaderStageFlagBits::eMissKHR; }
  else if (ext == "rint") { return vk::ShaderStageFlagBits::eIntersectionKHR; }
  else if (ext == "rcall") { return vk::ShaderStageFlagBits::eCallableKHR; }
  else if (ext == "mesh") { return vk::ShaderStageFlagBits::eMeshEXT; }
  else if (ext == "task") { return vk::ShaderStageFlagBits::eTaskEXT; }

  throw std::runtime_error("File extension `" + ext
                           + "` does not have a vulkan shader stage.");
}
}  //namespace

vk::ShaderModule VkShaderUtil::loadShader(vk::Device             device,
                                          uint32_t*              src,
                                          vk::DeviceSize         srcSize,
                                          std::vector<uint32_t>& spirv) {
  vk::ShaderModuleCreateInfo shaderCI;
  shaderCI.codeSize = srcSize;
  shaderCI.pCode    = src;

  spirv.resize(srcSize / sizeof(uint32_t));
  memcpy(spirv.data(), src, srcSize);
  vk::ShaderModule shader = device.createShaderModule(shaderCI);
  return shader;
}

vk::ShaderModule VkShaderUtil::loadShader(vk::Device             device,
                                          const std::string&     filename,
                                          std::vector<uint32_t>& spirv) {
  std::string              src   = readFileAsString(filename);
  std::vector<std::string> lines = replaceInclude(src);
  std::vector<uint8_t>     data  = convertStringsToBytes(lines);

  //Extract extension name from the glsl shader file
  std::string file_ext = filename;
  file_ext             = file_ext.substr(file_ext.find_last_of(".") + 1);

  return compileShader(device, find_shader_stage(file_ext), data, "main", {}, spirv);
}

vk::ShaderModule VkShaderUtil::loadShader(vk::Device              device,
                                          const std::string&      fileContents,
                                          vk::ShaderStageFlagBits stage,
                                          std::vector<uint32_t>&  spirv) {
  std::vector<std::string> lines = replaceInclude(fileContents);
  std::vector<uint8_t>     data  = convertStringsToBytes(lines);
  //std::vector<uint8_t>     data;
  //data.resize(fileContents.length());
  //memcpy(data.data(), fileContents.c_str(), data.size());
  return compileShader(device, stage, data, "main", {}, spirv);
}

vk::ShaderModule VkShaderUtil::compileShader(vk::Device                  device,
                                             vk::ShaderStageFlagBits     stage,
                                             const std::vector<uint8_t>& src,
                                             const std::string&          entry_point,
                                             const ShaderVariant&        shader_variant,
                                             std::vector<uint32_t>&      spirv) {
  std::string  info_log;
  GLSLCompiler glsl_compiler;

  if (!glsl_compiler.compile_to_spirv(stage, src, "main", {}, spirv, info_log)) {
    VklLogE("Failed to compile shader from source, Error: {}", info_log.c_str());
    return VK_NULL_HANDLE;
  }

  VkShaderModule           shader_module;
  VkShaderModuleCreateInfo module_create_info{};
  module_create_info.sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  module_create_info.codeSize = spirv.size() * sizeof(uint32_t);
  module_create_info.pCode    = spirv.data();

  VK_CHECK_ERROR(vkCreateShaderModule(device, &module_create_info, NULL, &shader_module),
                 "shader module creation fail");

  return shader_module;
}

std::vector<std::string> VkShaderUtil::replaceInclude(const std::string& src) {
  std::string              line_;
  std::vector<std::string> lines;
  char                     delim = '\n';

  std::stringstream sstream(src);

  while (std::getline(sstream, line_, delim)) { lines.push_back(line_ + "\n"); }

  std::vector<std::string> outs;

  for (auto& line : lines) {
    if (line.find("#include \"") == 0) {
      std::string headerFile = line.substr(10);
      size_t      last_quote = headerFile.find("\"");
      if (!headerFile.empty() && last_quote != std::string::npos) {
        headerFile = ResourcePool::getShaderPath() + "/"
                     + headerFile.substr(0, last_quote);
      }
      std::string headerSrc    = readFileAsString(headerFile);
      auto        intermediate = replaceInclude(headerSrc);
      for (auto& interLine : intermediate) { outs.push_back(interLine); }
    }
    else { outs.push_back(line); }
  }

  return outs;
}

std::string VkShaderUtil::readFileAsString(const std::string& fileName) {
  std::ifstream file;
  file.open(fileName, std::ios::in | std::ios::binary);

  if (!file.is_open()) { throw std::runtime_error("Failed to open file: " + fileName); }

  return std::string((std::istreambuf_iterator<char>(file)),
                     (std::istreambuf_iterator<char>()));
}

std::vector<uint8_t> VkShaderUtil::convertStringsToBytes(std::vector<std::string>& src) {
  std::vector<uint8_t> bytes;

  for (const std::string& line : src) {
    //Append the raw bytes of the string to the bytes vector
    bytes.insert(bytes.end(), line.begin(), line.end());
  }

  return bytes;
}
}  //namespace vkl