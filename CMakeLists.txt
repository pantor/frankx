cmake_minimum_required(VERSION 3.6)


project(frankx VERSION 0.2.0 LANGUAGES CXX)
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)


option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_MODULE "Build python module" ON)
option(BUILD_TESTS "Build tests" ON)
option(BUILD_PYBIND11 "Build pybind11 with cmake" OFF)
option(USE_PYTHON_EXTENSION "Use python in frankx library" ON)

set(REFLEXXES_TYPE "ReflexxesTypeII" CACHE STRING "Type of Reflexxes library") # or ReflexxesTypeIV


find_package(Eigen3 3.3.7 EXACT REQUIRED NO_MODULE)
find_package(Franka 0.6.0 REQUIRED)
find_package(Reflexxes REQUIRED)


add_library(libfrankx SHARED
  src/affine.cpp
  src/gripper.cpp
  src/motion_joint.cpp
  src/reaction.cpp
  src/robot.cpp
)
target_compile_features(libfrankx PUBLIC cxx_std_11)
target_compile_definitions(libfrankx PUBLIC AFFINE_WITH_ROBOT_CONTROL)
target_include_directories(libfrankx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(libfrankx PUBLIC Franka::Franka Reflexxes::Reflexxes Eigen3::Eigen)


if(BUILD_PYTHON_MODULE)
  if(BUILD_PYBIND11)
    add_subdirectory(pybind11)
  else()
    find_package(pybind11 REQUIRED)
  endif()

  if(USE_PYTHON_EXTENSION)
    target_compile_definitions(libfrankx PUBLIC WITH_PYTHON)
    target_link_libraries(libfrankx PUBLIC pybind11::embed)
  endif()

  pybind11_add_module(_frankx src/python.cpp)
  target_link_libraries(_frankx PUBLIC libfrankx)
endif()


if(BUILD_EXAMPLES)
  foreach(example IN ITEMS simple grasping)
    add_executable(${example} "examples/${example}.cpp")
    target_link_libraries(${example} PRIVATE libfrankx)
  endforeach()
endif()


if(BUILD_TESTS)
  enable_testing()

  find_package(Catch2 REQUIRED)

  foreach(test IN ITEMS unit-test robot-test)
    add_executable(${test} "test/${test}.cpp")
    target_link_libraries(${test} PRIVATE libfrankx Catch2::Catch2)
    add_test(${test} COMMAND ${test})
  endforeach()
endif()


install(TARGETS libfrankx LIBRARY DESTINATION lib INCLUDES DESTINATION include)
