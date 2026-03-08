include(FetchContent)

if(APPLE AND NOT DEFINED RERUN_DOWNLOAD_AND_BUILD_ARROW)
  # Prefer system Arrow on macOS to avoid long builds from source.
  set(RERUN_DOWNLOAD_AND_BUILD_ARROW OFF CACHE BOOL "Use system Arrow on macOS")
endif()

# Force external dependencies to build in Release mode for faster setup.
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)
if(BUILD_TYPE_LOWER STREQUAL "relwithdebinfo")
  set(CMAKE_BUILD_TYPE_SAVED "${CMAKE_BUILD_TYPE}")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type for external dependencies" FORCE)
endif()

FetchContent_Declare(
  rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

# Restore original build type for project code.
if(BUILD_TYPE_LOWER STREQUAL "relwithdebinfo")
  set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE_SAVED}" CACHE STRING "Build type" FORCE)
  unset(CMAKE_BUILD_TYPE_SAVED)
endif()

file(GLOB APP_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)
list(REMOVE_ITEM APP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/vo_main.cpp)

add_executable(euroc_vo ${CMAKE_CURRENT_SOURCE_DIR}/vo_main.cpp ${APP_SRC})
target_include_directories(euroc_vo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(euroc_vo PRIVATE toy::toy io toy_libs rerun_sdk)
target_compile_options(euroc_vo PRIVATE
  $<$<CONFIG:Release>:-O3>
  $<$<CONFIG:RelWithDebInfo>:-O2>
)
