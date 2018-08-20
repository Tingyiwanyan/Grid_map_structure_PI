
# Copyright (c) 
# Distributed under the MIT License.
# See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT


# opencv, NOTE: OpenCV vs OPENCV_*, case sensitive
find_package (OpenCV)
      #find_package (OpenCV REQUIRED imgproc highgui  imgcodecs)
      # cv::imwrite etc are in different libs, depending on lib versions?!
  if (OPENCV_IMGPROC_FOUND AND ( OPENCV_IMGCODECS_FOUND OR OPENCV_HIGHGUI_FOUND) )
    message("OpenCV REQUIRED MODULES FOUND")
    include_directories(${OpenCV_INCLUDE_DIRS})
  else()
    message(FATAL_ERROR "OpenCV REQUIRED MODULES *NOT* FOUND")
  endif()

# -lGL -lGLU -lglut -lXext, etc
find_package (OpenGL REQUIRED)
  if (OPENGL_FOUND)
    message("OPENGL FOUND")
    include_directories(${OpenGL_INCLUDE_DIRS})
  else (OPENGL_FOUND)
    message(FATAL_ERROR "OPENGL *NOT* FOUND")
  endif (OPENGL_FOUND)

find_package (GLUT REQUIRED)
  if (GLUT_FOUND)
    message("GLUT FOUND")
    include_directories(${GLUT_INCLUDE_DIRS})
  else (GLUT_FOUND)
    message(FATAL_ERROR "GLUT *NOT* FOUND")
  endif (GLUT_FOUND)

# armadillo
find_package (Armadillo REQUIRED)
  if (ARMADILLO_FOUND)
    message("ARMADILLO FOUND " ${ARMADILLO_LIBRARIES})
    include_directories(${ARMADILLO_INCLUDE_DIRS})
  else()
    message(FATAL_ERROR "ARMADILLO *NOT* FOUND")
  endif()

# pthreads
find_package (Threads REQUIRED)
  if (THREADS_FOUND)
    message("THREADS FOUND")
  else()
    message(SEND_ERROR "THREADS *NOT* FOUND")
  endif()

# netcdf_c++4
find_library(NETCDF_C++4_LIB
    NAMES ${LIBS_NETCDF_C++}
    PATHS /usr/local/lib /usr/lib
)
  if (NOT NETCDF_C++4_LIB)
    MESSAGE(FATAL_ERROR "NETCDF_C++4 *NOT* FOUND " ${NETCDF_C++4_LIB})
  else()
    MESSAGE("NETCDF_C++4 FOUND " ${NETCDF_C++4_LIB})
  endif()

# yaml-cpp
find_library(YAML_CPP_LIB
    NAMES ${LIBS_YAML_CPP}
    PATHS /usr/local/lib /usr/lib
)
  if (NOT YAML_CPP_LIB)
    MESSAGE(FATAL_ERROR "YAML_CPP *NOT* FOUND ")
  else()
    MESSAGE("YAML_CPP FOUND " ${YAML_CPP_LIB})
  endif()

# sogp
find_library(SOGP_LIB
    NAMES ${LIBS_SOGP}
    PATHS /usr/local/lib /usr/lib
)
  if (NOT SOGP_LIB)
    MESSAGE(SEND_ERROR "SOGP *NOT* FOUND ")
  else()
    MESSAGE("SOGP FOUND " ${SOGP_LIB})
  endif()

# boost
find_library(BOOST_FILESYSTEM_LIB
    NAMES ${LIBS_BOOST_FILESYSTEM}
    PATHS /usr/local/lib /usr/lib
)
  if (NOT BOOST_FILESYSTEM_LIB)
    MESSAGE(SEND_ERROR "BOOST_FILESYSTEM *NOT* FOUND ")
  else()
    MESSAGE("BOOST_FILESYSTEM FOUND " ${BOOST_FILESYSTEM_LIB})
  endif()

find_library(BOOST_SYSTEM_LIB
    NAMES ${LIBS_BOOST_SYSTEM}
    PATHS /usr/local/lib /usr/lib
)
  if (NOT BOOST_SYSTEM_LIB)
    MESSAGE(SEND_ERROR "BOOST_SYSTEM *NOT* FOUND ")
  else()
    MESSAGE("BOOST_SYSTEM FOUND " ${BOOST_SYSTEM_LIB})
  endif()



