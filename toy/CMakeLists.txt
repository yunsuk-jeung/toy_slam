cmake_minimum_required(VERSION 3.18.1)

project("toyslam")

file(GLOB_RECURSE TOY_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(${PROJECT_NAME} STATIC ${TOY_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})