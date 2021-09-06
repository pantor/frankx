cmake_minimum_required(VERSION 3.11)


project(frankx VERSION 0.1.1 LANGUAGES CXX)


include(GNUInstallDirs)


option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_MODULE "Build python module" ON)
option(BUILD_TESTS "Build tests" ON)
option(USE_PYTHON_EXTENSION "Use python in frankx library" ON)


find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(Franka 0.8 REQUIRED)

message("Found Eigen Version: ${Eigen3_VERSION}")
message("Found Franka Version: ${Franka_VERSION}")


add_subdirectory(affx)
add_subdirectory(ruckig)


add_library(movex src/movex/path.cpp)
target_compile_features(movex PUBLIC cxx_std_17)
target_include_directories(movex PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(movex PUBLIC affx ruckig)


add_library(frankx
  src/frankx/gripper.cpp
  src/frankx/kinematics.cpp
  src/frankx/robot.cpp
)
target_compile_features(frankx PUBLIC cxx_std_17)
target_include_directories(frankx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(frankx PUBLIC Franka::Franka Eigen3::Eigen movex)


if(BUILD_PYTHON_MODULE)
  # Check if pybind11 exists as a subdirectory
  if(EXISTS pybind11)
    add_subdirectory(pybind11)
  else()
    find_package(Python COMPONENTS Interpreter Development)
    find_package(pybind11 2.6 REQUIRED)
  endif()

  if(USE_PYTHON_EXTENSION)
    target_compile_definitions(frankx PUBLIC WITH_PYTHON)
    target_link_libraries(frankx PUBLIC pybind11::embed)
  endif()

  pybind11_add_module(_frankx src/frankx/python.cpp)
  target_link_libraries(_frankx PUBLIC frankx)

  # Movex python package for development
  pybind11_add_module(_movex src/movex/python.cpp)
  target_compile_features(_movex PUBLIC cxx_std_17)
  target_include_directories(_movex PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include pybind11::embed)
  target_link_libraries(_movex PUBLIC Eigen3::Eigen movex)
endif()


if(BUILD_EXAMPLES)
  foreach(example IN ITEMS linear grasping)
    add_executable(${example} "examples/${example}.cpp")
    target_link_libraries(${example} PRIVATE frankx)
  endforeach()
endif()


if(BUILD_TESTS)
  enable_testing()

  find_package(Catch2 REQUIRED)

  foreach(test IN ITEMS kinematics-test unit-test path-test)
    add_executable(${test} "test/${test}.cpp")
    target_link_libraries(${test} PRIVATE frankx Catch2::Catch2)
    add_test(NAME ${test} COMMAND ${test})
  endforeach()
endif()


install(TARGETS frankx
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
