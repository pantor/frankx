cmake_minimum_required(VERSION 3.5)


set(CMAKE_C_COMPILER "gcc-9")
set(CMAKE_CXX_COMPILER "g++-9")


project(frankx VERSION 0.2.0 LANGUAGES C CXX)


option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_MODULE "Build python module" ON)
option(BUILD_TEST "Build tests" ON)


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 17)


set(Eigen3_DIRS "/usr/local/include/eigen3-2/")
set(Reflexxes_LIB_DIR "/home/berscheid/Documents/libs/ReflexxesTypeIV/Linux/x64/release/lib/shared" CACHE STRING "Path to Reflexxes library")
set(Reflexxes_INCLUDE_DIR "/home/berscheid/Documents/libs/ReflexxesTypeIV/include" CACHE STRING "Path to Reflexxes headers")
set(Reflexxes_TYPE "ReflexxesTypeIV" CACHE STRING "Type of Reflexxes library")
set(PYTHON_EXECUTABLE python3.6)
set(PYTHON_LIBRARY "/usr/lib/python3.6/config-3.6m-x86_64-linux-gnu/libpython3.6m.so")


# find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Franka 0.6.0 REQUIRED)
find_library(Reflexxes ${Reflexxes_TYPE} PATHS ${Reflexxes_LIB_DIR})
find_package(pybind11 REQUIRED)
find_package(Python 3.6 EXACT REQUIRED)


add_library(libfrankx SHARED
  src/affine.cpp
  src/gripper.cpp
  src/motion_joint.cpp
  src/robot.cpp
)
target_include_directories(libfrankx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${Reflexxes_INCLUDE_DIR} ${Eigen3_DIRS})
target_link_libraries(libfrankx Franka::Franka ${Reflexxes})


if(BUILD_PYTHON_MODULE)
  pybind11_add_module(frankx src/python.cpp)
  target_link_libraries(frankx PUBLIC libfrankx)
endif()


if(BUILD_EXAMPLES)
  foreach(example IN ITEMS grasping)
    add_executable(${example} "examples/${example}.cpp")
    target_link_libraries(${example} libfrankx)
  endforeach()
endif()


if(BUILD_TEST)
  find_package(Catch2 REQUIRED)

  add_executable(unit_test "test/unit-test.cpp")
  target_link_libraries(unit_test libfrankx Catch2::Catch2)
endif()
