
cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_UTILS})
project(${PROJ_NAME})

#include_directories(/usr/local/include/gp)

set(PROJ_PATH ${SRC_PATH}/${PROJ_NAME})

set(${PROJ_NAME}_SRCS 
  argparser.c
  parameters.cc
  clock.cc
  data_loader.cc
)

add_library(${PROJ_NAME} STATIC ${${PROJ_NAME}_SRCS})

target_link_libraries(${PROJ_NAME} 
  ${OpenCV_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ${LIBS_NETCDF_C++}
  ${LIBS_YAML_CPP}
  ${LIBS_GP}
) 



#------------------------
##link libs

## opencv
#find_package (OpenCV REQUIRED core imgproc highgui video) 
#  if (OPENCV_CORE_FOUND AND OPENCV_HIGHGUI_FOUND AND OPENCV_IMGPROC_FOUND AND OPENCV_VIDEO_FOUND) 
#    message("OPENCV REQUIRED MODULES FOUND") 
#    include_directories(${OpenCV_INCLUDE_DIRS}) 
#    target_link_libraries(${PROJ_NAME} ${OpenCV_LIBRARIES}) 
#  else() 
#    message("OPENCV REQUIRED MODULES *NOT* FOUND") 
#  endif()
#
## armadillo
#find_package (Armadillo REQUIRED) 
#  if (ARMADILLO_FOUND) 
#    message("ARMADILLO FOUND " ${ARMADILLO_LIBRARIES}) 
#    include_directories(${ARMADILLO_INCLUDE_DIRS}) 
#    target_link_libraries(${PROJ_NAME} ${ARMADILLO_LIBRARIES}) 
#  else() 
#    message("ARMADILLO *NOT* FOUND") 
#  endif()
#
## pthreads
#find_package (Threads REQUIRED)
#  if (THREADS_FOUND) 
#    message("THREADS FOUND") 
#    target_link_libraries(${PROJ_NAME} ${CMAKE_THREAD_LIBS_INIT}) #note the difference 
#  else() 
#    message("THREADS *NOT* FOUND") 
#  endif()
#
## netcdf_c++4
#find_library(NETCDF_C++4_LIB
#    NAMES ${LIBS_NETCDF_C++}
#    PATHS /usr/local/lib /usr/lib
#)
#if (NOT NETCDF_C++4_LIB)
#  MESSAGE("NETCDF_C++4 *NOT* FOUND " ${NETCDF_C++4_LIB})
#else()
#  MESSAGE("NETCDF_C++4 FOUND " ${NETCDF_C++4_LIB})
#  target_link_libraries(${PROJ_NAME} ${LIBS_NETCDF_C++}) 
#endif()
#
## yaml-cpp
#find_library(YAML_CPP_LIB
#    NAMES ${LIBS_YAML_CPP}
#    PATHS /usr/local/lib /usr/lib
#)
#if (NOT YAML_CPP_LIB)
#  MESSAGE("YAML_CPP *NOT* FOUND ")
#else()
#  MESSAGE("YAML_CPP FOUND " ${YAML_CPP_LIB})
#  target_link_libraries(${PROJ_NAME} ${LIBS_YAML_CPP}) 
#endif()
#
## gp
#find_library(GP_LIB
#    NAMES ${LIBS_GP}
#    PATHS /usr/local/lib /usr/lib
#)
#if (NOT GP_LIB)
#  MESSAGE("GP *NOT* FOUND ")
#else()
#  MESSAGE("GP FOUND " ${GP_LIB})
#  target_link_libraries(${PROJ_NAME} ${LIBS_GP}) 
#endif()


#-------------------------------------
## commented blocks

##[[
#find_package (NetCDF REQUIRED)
#  if (NETCDF_FOUND) 
#    message("NETCDF FOUND") 
#    message(${NETCDF_LIBRARIES} " and " ${NETCDF_LIBRARIES_C})
#    target_link_libraries(${PROJ_NAME} ${NETCDF_LIBRARIES} ${NETCDF_LIBRARIES_C} netcdf_c++4) 
#    #target_link_libraries (uses_f90_interface ${NETCDF_LIBRARIES})
#    #target_link_libraries (only_uses_c_interface ${NETCDF_LIBRARIES_C})
#  else () 
#    message("NETCDF *NOT* FOUND") 
#  endif () 
#
#find_package (Lapack REQUIRED) 
#  if (LAPACK_FOUND) 
#    message("LAPACK FOUND") 
#    include_directories(${LAPACK_INCLUDE_DIRS}) 
#    target_link_libraries(${PROJ_NAME} ${LAPACK_LIBRARIES}) 
#  else (LAPACK_FOUND) 
#    message("LAPACK *NOT* FOUND") 
#  endif (LAPACK_FOUND)
#
#find_package (Blas REQUIRED) 
#  if (BLAS_FOUND) 
#    message("BLAS FOUND") 
#    include_directories(${BLAS_INCLUDE_DIRS}) 
#    target_link_libraries(${PROJ_NAME} ${BLAS_LIBRARIES}) 
#  else (BLAS_FOUND) 
#    message("BLAS *NOT* FOUND") 
#  endif (BLAS_FOUND)
#]]



