cmake_minimum_required(VERSION 3.18.1)

project("toy_samples")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_executable(sample_optflow sample_optflow.cpp)
target_link_libraries(sample_optflow PRIVATE toy::toy)