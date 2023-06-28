cmake_minimum_required(VERSION 3.11)


project(franky VERSION 0.2.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)


include(GNUInstallDirs)


option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON_MODULE "Build python module" ON)
option(BUILD_TESTS "Build tests" ON)


find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(Franka 0.8 REQUIRED)

message("Found Eigen Version: ${Eigen3_VERSION}")
message("Found Franka Version: ${Franka_VERSION}")

add_subdirectory(ruckig)

file(GLOB_RECURSE SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(franky ${SRC})
target_include_directories(franky PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(franky PUBLIC Franka::Franka Eigen3::Eigen ruckig)


if (BUILD_PYTHON_MODULE)
    # Check if pybind11 exists as a subdirectory
    if (EXISTS pybind11)
        add_subdirectory(pybind11)
    else ()
        find_package(Python COMPONENTS Interpreter Development)
        find_package(pybind11 2.6 REQUIRED)
    endif ()

    file(GLOB_RECURSE PYTHON_SRC "${CMAKE_CURRENT_SOURCE_DIR}/python/*.cpp")
    pybind11_add_module(_franky ${PYTHON_SRC})
    target_link_libraries(_franky PUBLIC franky)
endif ()


if (BUILD_EXAMPLES)
    foreach (example IN ITEMS linear)
        add_executable(${example} "examples/${example}.cpp")
        target_link_libraries(${example} PRIVATE franky)
    endforeach ()
endif ()


if (BUILD_TESTS)
    enable_testing()

    find_package(Catch2 REQUIRED)

    foreach (test IN ITEMS kinematics-test path-test)
        add_executable(${test} "test/${test}.cpp")
        target_link_libraries(${test} PRIVATE franky Catch2::Catch2)
        add_test(NAME ${test} COMMAND ${test})
    endforeach ()
endif ()


install(TARGETS franky
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )