cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_DISTURBANCE})
project(${PROJ_NAME})

set(${PROJ_NAME}_SRCS 
  disturbance.cc
)

add_library(${PROJ_NAME} STATIC ${${PROJ_NAME}_SRCS})

