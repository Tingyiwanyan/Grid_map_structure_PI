
CC	= g++  
CFLAGS 	= -g #-O3 -Wall -fPIC 
LD	= g++
LDFLAGS	= 
AR	= ar
ARFLAGS	= rs
RANLIB	= ranlib
RM	= rm
ECHO	= @echo

SHELL	= /bin/sh

INCL_DIR = -I/usr/local/lib -I/usr/local/include/eigen3 -I/usr/include/eigen3
DEPEND   = .depend    #each sub-dir will build a .depend dir to track dependacy

LIBS_GL   =  -lGLU -lglut -lGL -lXext -lpthread
LIBS_ARMA = -larmadillo -llapack -lblas #-llapack_atlas


# naming packages/directories (libs will use the names as well)

HUNGARIAN    = hungarian
ANIMATION    = animation
MDP_METHODS  = mdp_methods
INFO_METHODS = info_methods
VIZ_TOOL     = viz_tool
SPLINE       = spline
UTILS	     = utils
VEHICLE      = vehicle
DISTURBANCE  = disturbance

LIB_HUNGARIAN	 = lib$(HUNGARIAN).a
LIB_ANIMATION    = lib$(ANIMATION).a
LIB_MDP_METHODS  = lib$(MDP_METHODS).a
LIB_INFO_METHODS = lib$(INFO_METHODS).a
LIB_VIZ_TOOL     = lib$(VIZ_TOOL).a
LIB_SPLINE       = lib$(SPLINE).a
LIB_UTILS        = lib$(UTILS).a
LIB_VEHICLE      = lib$(VEHICLE).a
LIB_DISTURBANCE	 = lib$(DISTURBANCE).a


#uncomment below to silent output
#.SILENT :

#basics: http://owen.sj.ca.us/~rk/howto/slides/make/slides/makerecurs.html
#depend: http://stackoverflow.com/questions/2394609/makefile-header-dependencies

