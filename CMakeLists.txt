cmake_minimum_required(VERSION 3.18.1)

project("toy_project")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_TEST "Build test" OFF)
option(BUILD_SAMPLES "Build samples" ON)

set(EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib)
add_subdirectory(${EXTERNAL_LIB_DIR} ${CMAKE_BINARY_DIR}/lib)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/toy ${CMAKE_BINARY_DIR}/toy)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/io ${CMAKE_BINARY_DIR}/io)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libvulkanlight ${CMAKE_BINARY_DIR}/libvulkanlight)

if(BUILD_SAMPLES)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/samples)
endif()

if(BUILD_TEST)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Test ${CMAKE_BINARY_DIR}/Test)
endif()

add_executable(euroc_vio exec/vio_main.cpp)
target_link_libraries(euroc_vio PRIVATE toy::toy libs io)

if(WIN32)
  set(EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib)

  include(${EXTERNAL_LIB_DIR}/copyDLL.cmake)
  copy_tbb_dlls(euroc_vio ${EXTERNAL_LIB_DIR})
  copy_cv34_dlls(euroc_vio ${EXTERNAL_LIB_DIR})
endif()