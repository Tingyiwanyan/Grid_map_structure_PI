cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_SPLINE})
project(${PROJ_NAME})

set(${PROJ_NAME}_SRCS 
  UnconstrainedSplineFit.cc
  spline_fit.cc
)

include_directories(
  ${INCLUDE_PATH}
)

link_directories(
)

add_library(${PROJ_NAME} STATIC ${${PROJ_NAME}_SRCS})

target_link_libraries(${PROJ_NAME}
  ${LIBS_ARMA} 
)

