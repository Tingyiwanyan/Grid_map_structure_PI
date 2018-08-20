cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_INFO_METHODS})
project(${PROJ_NAME})

set(${PROJ_NAME}_SRCS 
  info_core.cc 
  tsp_brute.cc 
  gp_model.cc 
  dp_solver.cc 
  greedy_solver.cc
  random_solver.cc

  gp_model_old.cc 
)

include_directories(
  ${INCLUDE_PATH}
)

link_directories(
  ${PROJECT_PATH}/${PROJ_UTILS}
)

add_library(${PROJ_NAME} STATIC ${${PROJ_NAME}_SRCS})

target_link_libraries(${PROJ_NAME}
  ${PROJ_UTILS}
  ${LIBS_SOGP}
)


