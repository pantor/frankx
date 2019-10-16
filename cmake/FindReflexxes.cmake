if(NOT TARGET Reflexxes::Reflexxes)
  set(Reflexxes_LIB_DIR "${Reflexxes_ROOT_DIR}/Linux/x64/release/lib/shared")
  set(Reflexxes_INCLUDE_DIR "${Reflexxes_ROOT_DIR}/include")

  find_library(Reflexxes ${REFLEXXES_TYPE} PATHS ${Reflexxes_LIB_DIR})

  add_library(Reflexxes::Reflexxes INTERFACE IMPORTED)
  target_include_directories(Reflexxes::Reflexxes INTERFACE ${Reflexxes_INCLUDE_DIR})
  target_link_libraries(Reflexxes::Reflexxes INTERFACE ${Reflexxes})
endif()
