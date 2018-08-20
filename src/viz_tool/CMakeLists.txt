
cmake_minimum_required(VERSION 2.6)

set(PROJ_NAME ${PROJ_VIZ_TOOL})
project(${PROJ_NAME})

#set_source_files_properties(*.c PROPERTIES LANGUAGE CXX )

set(${PROJ_NAME}_SRCS 
  argparser.cc
  bmploader.cc
  canvas.cc
  floor.cc
  glm.cc
  gltb.cc
  glfunc.cc
  gldraw.cc
  shadow.cc
  texture.cc
  tex_walls.cc
)

# disable warnings from old c code
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    #set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS         "-w")  #disable warnings
endif(CMAKE_COMPILER_IS_GNUCXX)

add_library(${PROJ_NAME} STATIC ${${PROJ_NAME}_SRCS})

# need -lGL -lGLU -lglut -lXext, etc
target_link_libraries(${PROJ_NAME} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})


#[[
#find_package (opengL REQUIRED)
#  if (opengl_found)
#    message("opengl FOUND")
#    include_directories(${OPENGL_INCLUDE_DIRS})
#    target_link_libraries(${PROJ_NAME} ${OPENGL_LIBRARIES})
#  else (opengl_founD)
#    message("opengl *NOT* FOUND")
#  endif (opengl_fouND)
#
#find_package (glut REQUIRED)
#  if (glut_found)
#    message("glut fOUND")
#    include_directories(${GLUT_INCLUDE_DIRS})
#    target_link_libraries(${PROJ_NAME} ${GLUT_LIBRARIES})
#  else (glut_found)
#    message("glut *NOT* FOUND")
#  endif (glut_found)
#]]

