cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_MDP_METHODS})
project(${PROJ_NAME})

set(${PROJ_NAME}_SRCS
  mdp_core.cc
  SSA.cc
  ssp.cc
)

include_directories(
  ${INCLUDE_PATH}
)

link_directories(
  ${PROJECT_PATH}/${PROJ_SPLINE}
)

add_library(${PROJ_NAME} STATIC ${${PROJ_NAME}_SRCS})

target_link_libraries(${PROJ_NAME}
   ${PROJ_SPLINE}
)
