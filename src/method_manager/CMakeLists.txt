cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_METHOD_MANAGER})
project(${PROJ_NAME})

set(${PROJ_NAME}_SRCS 
  method_manager.cc
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

  ${LIBS_BOOST_SYSTEM} ${LIBS_BOOST_FILESYSTEM}
)

