if(NOT TARGET Reflexxes::Reflexxes)
  set(Reflexxes_TYPE "ReflexxesTypeIV" CACHE STRING "Type of Reflexxes library") # or "ReflexxesTypeII"
  set(Reflexxes_LIB_DIR "~/Documents/libs/${Reflexxes_TYPE}/Linux/x64/release/lib/shared" CACHE STRING "Path to Reflexxes library")
  set(Reflexxes_INCLUDE_DIR "~/Documents/libs/${Reflexxes_TYPE}/include" CACHE STRING "Path to Reflexxes headers")
endif()