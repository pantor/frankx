cmake_minimum_required(VERSION 3.11)


project(frankx VERSION 0.1.0 LANGUAGES CXX)
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)


option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_MODULE "Build python module" ON)
option(BUILD_TESTS "Build tests" ON)
option(BUILD_PYBIND11 "Build pybind11 with cmake" OFF)
option(USE_PYTHON_EXTENSION "Use python in frankx library" ON)


find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(Franka 0.7 REQUIRED)
find_package(Reflexxes)


message("Found Eigen Version: ${Eigen3_VERSION}")
message("Found Franka Version: ${Franka_VERSION}")

if(Reflexxes)
  set(REFLEXXES_TYPE "ReflexxesTypeII" CACHE STRING "Type of Reflexxes library") # or ReflexxesTypeIV

  message("Found Reflexxes ${REFLEXXES_TYPE}")
else()
  message("Did not found Reflexxes.")
endif()


add_library(frankx SHARED
  src/movex/affine.cpp
  src/frankx/gripper.cpp
  src/frankx/motion_impedance.cpp
  src/frankx/robot.cpp
)
target_compile_features(frankx PUBLIC cxx_std_17)
target_include_directories(frankx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(frankx PUBLIC Franka::Franka Eigen3::Eigen)

if(Reflexxes)
  target_compile_definitions(frankx PUBLIC WITH_REFLEXXES)
  target_link_libraries(frankx PUBLIC Reflexxes::Reflexxes)
endif()


if(BUILD_PYTHON_MODULE)
  if(BUILD_PYBIND11)
    add_subdirectory(pybind11)
  else()
    find_package(Python COMPONENTS Interpreter Development)
    find_package(pybind11 2.6.0 REQUIRED)
  endif()

  if(USE_PYTHON_EXTENSION)
    target_compile_definitions(frankx PUBLIC WITH_PYTHON)
    target_link_libraries(frankx PUBLIC pybind11::embed)
  endif()

  pybind11_add_module(_frankx src/frankx/python.cpp)
  target_link_libraries(_frankx PUBLIC frankx)

  pybind11_add_module(movex src/movex/python.cpp)
  target_compile_features(movex PUBLIC cxx_std_17)
  target_include_directories(movex PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include pybind11::embed)
  target_link_libraries(movex PUBLIC Eigen3::Eigen)

  if(Reflexxes)
    target_compile_definitions(movex PUBLIC WITH_REFLEXXES)
    target_link_libraries(movex PUBLIC Reflexxes::Reflexxes)
  endif()
endif()


if(BUILD_EXAMPLES)
  foreach(example IN ITEMS simple grasping)
    add_executable(${example} "examples/${example}.cpp")
    target_link_libraries(${example} PRIVATE ${PROJECT_NAME})
  endforeach()
endif()


if(BUILD_TESTS)
  enable_testing()

  find_package(Catch2 REQUIRED)

  foreach(test IN ITEMS unit-test otg-test)
    add_executable(${test} "test/${test}.cpp")
    target_link_libraries(${test} PRIVATE ${PROJECT_NAME} Catch2::Catch2)
    add_test(${test} COMMAND ${test})
  endforeach()
endif()


install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib INCLUDES DESTINATION include)
