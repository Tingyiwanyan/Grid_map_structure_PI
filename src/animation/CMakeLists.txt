cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_ANIMATION})
project(${PROJ_NAME})

set(${PROJ_NAME}_SRCS 
  render_mdp.cc
  animation.cc
)

include_directories(
  ${INCLUDE_PATH}
)

link_directories(
  ${PROJECT_PATH}/${PROJ_METHOD_MANAGER}
)

add_library(${PROJ_NAME} STATIC ${${PROJ_NAME}_SRCS})

target_link_libraries(${PROJ_NAME}
  ${PROJ_METHOD_MANAGER}
)

