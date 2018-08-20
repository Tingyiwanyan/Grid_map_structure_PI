cmake_minimum_required (VERSION 2.6)

set(PROJECT_NAME aq_plan)
project(${PROJECT_NAME})

# configure a header file to pass some of the CMake settings
# to the source code
#configure_file (
#  "${PROJECT_SOURCE_DIR}/Config.h.in"
#  "${PROJECT_BINARY_DIR}/Config.h"
#  )

set(OpenCV_DIR /home/shoubhikdn/opencv-3.1.0/build/)

# macros/variables
include(define.cmake)

# configuring packages
include(packages.cmake)

# dependencies
include(dependencies.cmake)

# Set CMake build-type. If it not supplied by the user, the default built type is "Release".
if(((NOT CMAKE_BUILD_TYPE)
  AND (NOT BUILD_COVERAGE_ANALYSIS))
  OR (CMAKE_BUILD_TYPE STREQUAL "Release"))
    message(STATUS "WARNING: building release version!")
    set(CMAKE_BUILD_TYPE Release)
elseif((CMAKE_BUILD_TYPE STREQUAL "Debug") OR (BUILD_COVERAGE_ANALYSIS))
    message(STATUS "WARNING: building debug version!")
endif(((NOT CMAKE_BUILD_TYPE)
  AND (NOT BUILD_COVERAGE_ANALYSIS))
  OR (CMAKE_BUILD_TYPE STREQUAL "Release"))

if(CMAKE_COMPILER_IS_GNUCXX)
    MESSAGE(STATUS "GNUCXX Compiler")
    set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -fprofile-arcs -ftest-coverage")
endif(CMAKE_COMPILER_IS_GNUCXX)

include_directories(AFTER 
  ${EIGEN_INCL_PATH}
  ${SOGP_INCL_PATH}
  ${INCLUDE_PATH} 
)

add_subdirectory(${SRC_PATH}/${PROJ_UTILS})
add_subdirectory(${SRC_PATH}/${PROJ_SPLINE})
add_subdirectory(${SRC_PATH}/${PROJ_VIZ_TOOL})
add_subdirectory(${SRC_PATH}/${PROJ_DISTURBANCE})
add_subdirectory(${SRC_PATH}/${PROJ_VEHICLE})
add_subdirectory(${SRC_PATH}/${PROJ_MDP_METHODS})
add_subdirectory(${SRC_PATH}/${PROJ_INFO_METHODS})
add_subdirectory(${SRC_PATH}/${PROJ_HUNGARIAN})
add_subdirectory(${SRC_PATH}/${PROJ_METHOD_MANAGER})
add_subdirectory(${SRC_PATH}/${PROJ_ANIMATION})

link_directories(
  ${EXTERNAL_LIBS_PATH}
  ${SRC_PATH}/${PROJ_UTILS}
  ${SRC_PATH}/${PROJ_SPLINE}
  ${SRC_PATH}/${PROJ_VIZ_TOOL}
  ${SRC_PATH}/${PROJ_DISTURBANCE}
  ${SRC_PATH}/${PROJ_VEHICLE}
  ${SRC_PATH}/${PROJ_MDP_METHODS}
  ${SRC_PATH}/${PROJ_INFO_METHODS}
  ${SRC_PATH}/${PROJ_METHOD_MANAGER}
  ${SRC_PATH}/${PROJ_ANIMATION}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIB_PATH})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIB_PATH})

#set(LIB_NAME "${PROJECT_NAME}")
#add_library(${LIB_NAME} STATIC ${MAIN_SRC})

if(BUILD_MAIN)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_PATH})
  add_executable(${MAIN_NAME} ${MAIN_SRC})
  target_link_libraries(${MAIN_NAME} 
	${PROJS_ALL}
#	${LIBS_ALL}
#	${LIB_NAME}
  )

  add_executable(ocean_pred ${SRC_PATH}/main/ocean_pred.cc)
  target_link_libraries(ocean_pred ${PROJS_ALL})

  add_executable(ocean_planning ${SRC_PATH}/main/ocean_planning.cc)
  target_link_libraries(ocean_planning ${PROJS_ALL} ${LIBS_BOOST_SYSTEM} ${LIBS_BOOST_FILESYSTEM})

  add_executable(draw_prediction ${SRC_PATH}/main/draw_prediction.cc)
  target_link_libraries(draw_prediction ${PROJS_ALL})
endif(BUILD_MAIN)




#--------------------------------------------------
# other options

if(BUILD_DOXYGEN_DOCS)
  find_package(Doxygen)

  if(NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
            "Doxygen is needed to build the documentation. Please make sure it is installed.")
  endif(NOT DOXYGEN_FOUND)

  configure_file(${PROJECT_PATH}/Doxyfile.in
                 ${PROJECT_BINARY_DIR}/Doxyfile @ONLY IMMEDIATE)

  # Add a custom target to run Doxygen whenever the project is built.
  # If you do NOT want the documentation to be generated EVERY time you build the project
  # then leave out the 'ALL' keyword from the command below.
  add_custom_target(doxygen_docs ALL COMMAND ${DOXYGEN_EXECUTABLE}
                    ${PROJECT_BINARY_DIR}/Doxyfile
                    SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)
endif(BUILD_DOXYGEN_DOCS)


if(BUILD_TESTS)
  enable_testing()
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TEST_PATH})

  add_executable(${TEST_NAME} ${TEST_SRC})
  target_link_libraries(${TEST_NAME} ${LIB_NAME})
  add_test(NAME ${TEST_NAME} COMMAND "${TEST_PATH}/${TEST_NAME}")

  if(BUILD_COVERAGE_ANALYSIS)
    include(CodeCoverage)
    set(COVERAGE_EXTRACT '${PROJECT_PATH}/include/*' '${PROJECT_PATH}/src/*')
    setup_target_for_coverage(coverage "${TEST_NAME}" coverage)
  endif(BUILD_COVERAGE_ANALYSIS)
endif(BUILD_TESTS)

# Install header files and library.
# Destination is set by CMAKE_INSTALL_PREFIX and defaults to usual locations, unless overridden by
# user.
install(DIRECTORY ${INCLUDE_PATH}/${CMAKE_PROJECT_NAME}
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp")
install(TARGETS ${LIB_NAME} DESTINATION lib)
install(TARGETS ${BIN_NAME} DESTINATION bin)


                                               
