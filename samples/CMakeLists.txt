cmake_minimum_required(VERSION 3.18.1)

project("samples")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# add_executable(sample_optflow sample_optflow.cpp)
# target_link_libraries(sample_optflow PRIVATE toy::toy)
# target_link_libraries(sample_optflow PRIVATE io)
add_executable(sample_vulkan sample_vulkan.cpp )
add_executable(sample_custom_fmt "sample_custom_fmt.cpp")
add_executable(sample_sparse_matrix_draw "sample_sparse_matrix_draw.cpp")
add_executable(sample_tbb "sample_tbb.cpp")

target_compile_options(sample_vulkan PUBLIC
  $<$<CONFIG:Debug>:/MDd>
  $<$<NOT:$<CONFIG:Debug>>:/MD>
)

target_link_libraries(sample_vulkan PRIVATE libs vulkanLight::vulkanLight)
target_link_libraries(sample_custom_fmt PRIVATE libs)
target_link_libraries(sample_sparse_matrix_draw PRIVATE libs toy::toy)
target_link_libraries(sample_tbb PRIVATE libs)

# target_link_libraries(sample_optflow PRIVATE io)
# if(WIN32)
# set(EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)
# include(${EXTERNAL_LIB_DIR}/copyDLL.cmake)
# copy_tbb_dlls(sample_optflow ${EXTERNAL_LIB_DIR})
# copy_cv34_dlls(sample_optflow ${EXTERNAL_LIB_DIR})
# endif()
set(SAMPLE_OUT_DIR ${CMAKE_BINARY_DIR}/${PROJECT_NAME})
# message(FATAL_ERROR ${SAMPLE_OUT_DIR})

if(WIN32)
  set(EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)
  include(${EXTERNAL_LIB_DIR}/copyDLL.cmake)

  copy_tbb_dlls_to_dir(sample_sparse_matrix_draw ${EXTERNAL_LIB_DIR} ${SAMPLE_OUT_DIR})
  copy_cv34_dlls_to_dir(sample_sparse_matrix_draw ${EXTERNAL_LIB_DIR} ${SAMPLE_OUT_DIR})
endif()