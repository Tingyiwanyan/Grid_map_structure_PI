
cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_VEHICLE})
project(${PROJ_NAME})

set(${PROJ_NAME}_SRCS 
  vehicle_controller.cc
  vehicle_model.cc
)

include_directories(
  ${INCLUDE_PATH}
)

link_directories(
  ${PROJECT_PATH}/${PROJ_MDP_METHODS}
  ${PROJECT_PATH}/${PROJ_INFO_METHODS}
)

add_library(${PROJ_NAME} STATIC ${${PROJ_NAME}_SRCS})

target_link_libraries(${PROJ_NAME}
   ${PROJ_MDP_METHODS}
   ${PROJ_INFO_METHODS}
)

