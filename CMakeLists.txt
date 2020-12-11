cmake_minimum_required(VERSION 3.11)


project(frankx VERSION 0.1.0 LANGUAGES CXX)
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)


option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_MODULE "Build python module" ON)
option(BUILD_TESTS "Build tests" ON)
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


add_library(movex SHARED
  src/movex/affine.cpp
  src/movex/path.cpp
  src/movex/ruckig.cpp
)
target_compile_features(movex PUBLIC cxx_std_17)
target_include_directories(movex PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(movex PUBLIC Eigen3::Eigen)


add_library(frankx SHARED
  src/frankx/gripper.cpp
  src/frankx/robot.cpp
  src/frankx/robot_impedance_motion.cpp
  src/frankx/robot_joint_motion.cpp
  src/frankx/robot_path_motion.cpp
  src/frankx/robot_waypoint_motion.cpp
)
target_compile_features(frankx PUBLIC cxx_std_17)
target_include_directories(frankx PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(frankx PUBLIC Franka::Franka Eigen3::Eigen movex)

if(Reflexxes)
  target_compile_definitions(frankx PUBLIC WITH_REFLEXXES)
  target_link_libraries(frankx PUBLIC Reflexxes::Reflexxes)
endif()


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

  if(Reflexxes)
    target_compile_definitions(_movex PUBLIC WITH_REFLEXXES)
    target_link_libraries(_movex PUBLIC Reflexxes::Reflexxes)
  endif()
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

  foreach(test IN ITEMS unit-test otg-test path-test)
    add_executable(${test} "test/${test}.cpp")
    if(Reflexxes)
      target_compile_definitions(${test} PUBLIC WITH_REFLEXXES)
    endif()
    target_link_libraries(${test} PRIVATE frankx Catch2::Catch2)
    add_test(NAME ${test} COMMAND ${test})
  endforeach()
endif()


install(TARGETS frankx
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_PYTHON_MODULE)
  execute_process(
    COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE
  )
	install(TARGETS _frankx LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES})
endif()
