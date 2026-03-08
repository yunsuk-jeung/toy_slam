cmake_minimum_required(VERSION 3.18.1)

project("toy_project")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(BUILD_TEST "Build test" OFF)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party ${CMAKE_BINARY_DIR}/third_party)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_BINARY_DIR}/src)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/app ${CMAKE_BINARY_DIR}/app)

if(BUILD_TEST)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Test ${CMAKE_BINARY_DIR}/Test)
endif()
