cmake_minimum_required(VERSION 3.18.1)

project("toy_vio")

option(BUILD_TEST "Build test" OFF)

if(BUILD_TEST)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Test ${CMAKE_BINARY_DIR}/Test)
endif()

set(EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs)

option(LINK_WITH_EIGEN "link with eigen" ON)
option(LINK_WITH_TBB "link with tbb" ON)
add_subdirectory(${EXTERNAL_LIB_DIR} ${CMAKE_BINARY_DIR}/libs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_executable(example main.cpp)
target_link_libraries(example PUBLIC libs)

if(WIN32)

  include(${EXTERNAL_LIB_DIR}/copyDLL.cmake)
  set(BIN_DIR ${EXTERNAL_LIB_DIR}/libs_win/bin)
  copy_tbb_dlls(example ${BIN_DIR})

endif()