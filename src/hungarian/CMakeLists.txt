cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_HUNGARIAN})
project(${PROJ_NAME})

set(${PROJ_NAME}_SRCS 
  Assignment.cpp
  BipartiteGraph.cpp
  Hungarian.cpp
  PlotGraph.cpp
  Cgnuplot.cpp
  graph_hungarian.cpp
)


# disable warnings from old c code
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    #set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS         "-w")  #disable warnings
endif(CMAKE_COMPILER_IS_GNUCXX)


include_directories(
  ${INCLUDE_PATH}
)

link_directories( 
)

add_library(${PROJ_NAME} STATIC ${${PROJ_NAME}_SRCS})

target_link_libraries(${PROJ_NAME}
)


