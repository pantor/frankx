cmake_minimum_required(VERSION 3.5)


project(frankx VERSION 0.2.0 LANGUAGES C CXX)


list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)


option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_MODULE "Build python module" ON)
option(BUILD_TESTS "Build tests" ON)
set(REFLEXXES_TYPE "ReflexxesTypeII" CACHE STRING "Type of Reflexxes library") # or ReflexxesTypeIV


set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 17)


# find_package(Eigen3 3.3.7 REQUIRED)
find_package(Franka 0.6.0 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python 3.6 EXACT REQUIRED)
find_package(Reflexxes REQUIRED)


add_library(libfrankx SHARED
  src/affine.cpp
  src/condition.cpp
  src/gripper.cpp
  src/motion_joint.cpp
  src/robot.cpp
)
target_include_directories(libfrankx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(libfrankx Franka::Franka Reflexxes::Reflexxes)


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


if(BUILD_TESTS)
  enable_testing()

  find_package(Catch2 REQUIRED)

  add_executable(unit_test "test/unit-test.cpp")
  target_link_libraries(unit_test libfrankx Catch2::Catch2)
  add_test("unit-test" COMMAND unit_test)

  add_executable(robot_test "test/robot-test.cpp")
  target_link_libraries(robot_test libfrankx Catch2::Catch2)
  add_test("robot-test" COMMAND robot_test)
endif()
