cmake_minimum_required(VERSION 3.18.1)

project("toy_samples")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# add_executable(sample_optflow sample_optflow.cpp)
# target_link_libraries(sample_optflow PRIVATE toy::toy)
# target_link_libraries(sample_optflow PRIVATE io)
add_executable(sample_vulkan sample_vulkan.cpp)
target_compile_options(sample_vulkan PUBLIC
  $<$<CONFIG:Debug>:/MDd>
  $<$<NOT:$<CONFIG:Debug>>:/MD>
)
target_link_libraries(sample_vulkan PRIVATE libs vulkanLight::vulkanLight)

# target_link_libraries(sample_optflow PRIVATE io)
# if(WIN32)
# set(EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)
# include(${EXTERNAL_LIB_DIR}/copyDLL.cmake)
# copy_tbb_dlls(sample_optflow ${EXTERNAL_LIB_DIR})
# copy_cv34_dlls(sample_optflow ${EXTERNAL_LIB_DIR})
# endif()
if(WIN32)
  set(EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)
  include(${EXTERNAL_LIB_DIR}/copyDLL.cmake)
  copy_tbb_dlls(sample_vulkan ${EXTERNAL_LIB_DIR})
  copy_cv34_dlls(sample_vulkan ${EXTERNAL_LIB_DIR})
endif()