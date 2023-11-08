cmake_minimum_required(VERSION 3.18.1)

project("toy_sample")

option(BUILD_TEST "Build test" OFF)

if(BUILD_TEST)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Test ${CMAKE_BINARY_DIR}/Test)
endif()

set(EXTERNAL_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs)

option(LINK_WITH_EIGEN "link with eigen" ON)
option(LINK_WITH_TBB "link with tbb" ON)
option(LINK_WITH_SPDLOG "link with SPDLOG" ON)
option(LINK_WITH_JSON "link with JSON" ON)
option(LINK_WITH_OPENCV34 "link with cv34" ON)

add_subdirectory(${EXTERNAL_LIB_DIR} ${CMAKE_BINARY_DIR}/libs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/toy ${CMAKE_BINARY_DIR}/toy)

add_executable(example exec/vio_main.cpp)
target_link_libraries(example PRIVATE libs toyslam::toyslam)

if(WIN32)

	include(${EXTERNAL_LIB_DIR}/copyDLL.cmake)
  if(LINK_WITH_TBB)
	copy_tbb_dlls(example ${EXTERNAL_LIB_DIR})
  endif()

  if(LINK_WITH_OPENCV34)
	copy_cv34_dlls(example ${EXTERNAL_LIB_DIR})
  endif()
  
endif()