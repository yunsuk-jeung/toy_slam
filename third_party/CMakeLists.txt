cmake_minimum_required(VERSION 3.18.1)

project(toy_third_party LANGUAGES C CXX)

include(FetchContent)

set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/_deps CACHE PATH
  "Directory for FetchContent downloads")

option(TOYSLAM_FETCH_OPENCV "Fetch OpenCV from source" ON)
option(TOYSLAM_FETCH_TBB "Fetch oneTBB from source" ON)

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

# Eigen
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)

# fmt
set(FMT_TEST OFF CACHE BOOL "" FORCE)
set(FMT_DOC OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 10.2.1
)
FetchContent_MakeAvailable(fmt)

# spdlog (use external fmt)
set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG v1.13.0
)
FetchContent_MakeAvailable(spdlog)

# nlohmann json
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(nlohmann_json)

# Sophus
set(SOPHUS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SOPHUS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  sophus
  GIT_REPOSITORY https://github.com/strasdat/Sophus.git
  GIT_TAG 1.22.10
)
FetchContent_MakeAvailable(sophus)

# TBB
if(TOYSLAM_FETCH_TBB)
  set(TBB_TEST OFF CACHE BOOL "" FORCE)
  set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    oneTBB
    GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
    GIT_TAG v2021.12.0
  )
  FetchContent_MakeAvailable(oneTBB)
  if(TARGET tbb AND NOT TARGET TBB::tbb)
    add_library(TBB::tbb ALIAS tbb)
  endif()
else()
  find_package(TBB REQUIRED)
endif()

# OpenCV
if(TOYSLAM_FETCH_OPENCV)
  set(BUILD_LIST "core;imgproc;imgcodecs;highgui;video;features2d" CACHE STRING "" FORCE)
  set(BUILD_opencv_world OFF CACHE BOOL "" FORCE)
  set(BUILD_opencv_apps OFF CACHE BOOL "" FORCE)
  set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_PERF_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    opencv
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG 4.9.0
  )
  FetchContent_MakeAvailable(opencv)
  set(TOYSLAM_OPENCV_LIBS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_highgui
    opencv_video
    opencv_features2d
  )
  set(TOYSLAM_OPENCV_INCLUDE_DIRS
    ${opencv_SOURCE_DIR}/include
    ${opencv_BINARY_DIR}
  )
else()
  find_package(OpenCV REQUIRED)
  set(TOYSLAM_OPENCV_LIBS ${OpenCV_LIBS})
  set(TOYSLAM_OPENCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
endif()

add_library(toy_libs INTERFACE)
target_link_libraries(toy_libs INTERFACE
  Eigen3::Eigen
  fmt::fmt
  spdlog::spdlog
  nlohmann_json::nlohmann_json
  Sophus::Sophus
  ${TOYSLAM_OPENCV_LIBS}
  TBB::tbb
)
if(TOYSLAM_OPENCV_INCLUDE_DIRS)
  target_include_directories(toy_libs INTERFACE ${TOYSLAM_OPENCV_INCLUDE_DIRS})
endif()

add_library(toy_libs::toy_libs ALIAS toy_libs)
