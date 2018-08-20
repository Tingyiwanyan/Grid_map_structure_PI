# Copyright (c) 
# Distributed under the MIT License.
# See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT

set(PROJECT_PATH                               "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}     "${PROJECT_PATH}/cmake/modules")
set(INCLUDE_PATH                               "${PROJECT_PATH}/include")
set(SRC_PATH                                   "${PROJECT_PATH}/src")
set(TEST_SRC_PATH                              "${PROJECT_PATH}/test")
if(NOT EXTERNAL_PATH)
  set(EXTERNAL_PATH                            "${PROJECT_PATH}/external")
endif(NOT EXTERNAL_PATH)
if(NOT DOXYGEN_PATH)
  set(DOXYGEN_PATH                             "${PROJECT_PATH}/doxydocs")
endif(NOT DOXYGEN_PATH)
if(NOT DOCS_PATH)
  set(DOCS_PATH                                "${PROJECT_PATH}/docs")
endif(NOT DOCS_PATH)
set(LIB_PATH                                   "${PROJECT_BINARY_DIR}/lib")
#set(BIN_PATH                                   "${PROJECT_BINARY_DIR}/bin")
set(BIN_PATH                                   "${PROJECT_BINARY_DIR}")
set(MAIN_NAME                                  "${PROJECT_NAME}_exe")
set(TEST_PATH                                  "${PROJECT_BINARY_DIR}/test")
set(TEST_NAME                                  "test_${PROJECT_NAME}")

OPTION(BUILD_MAIN                              "Build main function"            ON)
OPTION(BUILD_DOXYGEN_DOCS                      "Build docs"                     OFF)
OPTION(BUILD_TESTS                             "Build tests"                    OFF)
OPTION(BUILD_DEPENDENCIES                      "Force build of dependencies"    OFF)

include(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(BUILD_COVERAGE_ANALYSIS "Build code coverage analysis"   OFF
                                               "BUILD_TESTS"                    OFF)
#projs
set(PROJ_METHOD_MANAGER "method_manager")
set(PROJ_ANIMATION	"animation")
set(PROJ_MDP_METHODS	"mdp_methods")
set(PROJ_INFO_METHODS	"info_methods")
set(PROJ_VIZ_TOOL	"viz_tool")
set(PROJ_SPLINE		"spline")
set(PROJ_UTILS		"utils")
set(PROJ_VEHICLE	"vehicle")
set(PROJ_DISTURBANCE	"disturbance")
set(PROJ_HUNGARIAN	"hungarian")
set(PROJS_ALL
  ${PROJ_METHOD_MANAGER} ${PROJ_ANIMATION} ${PROJ_MDP_METHODS} ${PROJ_INFO_METHODS}
  ${PROJ_VIZ_TOOL} ${PROJ_SPLINE} ${PROJ_UTILS} ${PROJ_VEHICLE}
  ${PROJ_DISTURBANCE} ${PROJ_HUNGARIAN}
)

#libs
set(LIBS_ARMA		"armadillo")  #lapack blas
set(LIBS_NETCDF_C++	"netcdf_c++4")
set(LIBS_YAML_CPP	"yaml-cpp")
set(LIBS_SOGP		"sogp")
set(LIBS_OPENGL		"GLU glut GL Xext pthread")
set(LIBS_OPENCV		"opencv_core opencv_highgui opencv_imgproc opencv_video") #-lopencv_imgcodecs
set(LIBS_BOOST_FILESYSTEM  "boost_filesystem")
set(LIBS_BOOST_SYSTEM      "boost_system")
set(LIBS_ALL		
  #${LIBS_ARMA}
  #${LIBS_NETCDF_C++} ${LIBS_YAML_CPP} ${LIBS_GP} 
  #${LIBS_OPENGL} ${LIBS_OPENCV}
)

set(EXTERNAL_LIBS_PATH
  "/usr/local/lib"
  "/usr/lib"
)

set(EXTERNAL_LIBS
)

set(SOGP_INCL_PATH
  "/usr/local/include/sogp"
)

set(EIGEN_INCL_PATH
  "/usr/local/include/eigen3"
  "/usr/include/eigen3"
)

set(MAIN_SRC
  ${SRC_PATH}/main/main.cc
)

set(TEST_SRC
  ${TEST_SRC_PATH}/test.cpp
)

# Set project source files.
#set(SRC
#  ${SRC_PATH}/*.cpp
#)


