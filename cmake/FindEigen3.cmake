if(NOT TARGET Eigen3::Eigen3)
  set(EIGEN3_INCLUDE_DIRS "/usr/local/include/eigen3-2/")

  add_library(Eigen3::Eigen3 INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS}
  )
endif()