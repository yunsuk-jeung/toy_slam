cmake_minimum_required(VERSION 3.18.1)

project("toy_vio")

option(BUILD_TEST "Build test" OFF)

if(BUILD_TEST)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Test ${CMAKE_BINARY_DIR}/Test)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_executable(example main.cpp)
