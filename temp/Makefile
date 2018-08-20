

include Makefile.inc

#CPLEX_FLAG = #-DIL_STD -lilocplex -lconcert -lcplex -lm 

DIRS = $(VEHICLE) $(DISTURBANCE) $(SPLINE) $(UTILS) $(VIZ_TOOL) $(MDP_METHODS) $(INFO_METHODS) $(ANIMATION) $(HUNGARIAN) 

OBJLIBS	= $(LIB_VEHICLE) $(LIB_DISTURBANCE) $(LIB_SPLINE) $(LIB_UTILS) $(LIB_VIZ_TOOL) $(LIB_MDP_METHODS) $(LIB_INFO_METHODS) $(LIB_ANIMATION) $(LIB_HUNGARIAN)

OBJS = #$(ANIMATION)/main.o 

# Note, linking order matters. Put the dep-intensive libs in front
# also, the 1st main in those libs will be used (?)
LD_LIBS	= -L$(ANIMATION) -l$(ANIMATION) \
	-L$(MDP_METHODS) -l$(MDP_METHODS) \
	-L$(INFO_METHODS) -l$(INFO_METHODS) \
	-L$(VEHICLE) -l$(VEHICLE) \
	-L$(DISTURBANCE) -l$(DISTURBANCE) \
	-L$(SPLINE) -l$(SPLINE) \
	-L$(VIZ_TOOL) -l$(VIZ_TOOL) \
	-L$(UTILS) -l$(UTILS) \
	-L$(HUNGARIAN) -l$(HUNGARIAN) \
	-L/usr/local/lib -lyaml-cpp\
	-lnetcdf_c++4 \
	-lgp \
	-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_imgcodecs


EXE	= simu
all : $(EXE)

$(EXE) : $(OBJS) $(OBJLIBS)
	$(LD) -o $(EXE) $(OBJS) $(LD_LIBS) $(LIBS_GL) $(LIBS_ARMA) 
	$(ECHO) Done making $(EXE).
	$(ECHO)

#libsub.a libsubsub.a : force_look
#	$(ECHO) looking into subdir : $(MAKE) $(MFLAGS)
#	cd subdir; $(MAKE) $(MFLAGS)

$(LIB_VEHICLE): force_look
	cd $(VEHICLE); $(MAKE) $(MFLAGS)

$(LIB_DISTURBANCE): force_look
	cd $(DISTURBANCE); $(MAKE) $(MFLAGS)

$(LIB_SPLINE): force_look
	cd $(SPLINE); $(MAKE) $(MFLAGS)

$(LIB_UTILS): force_look
	cd $(UTILS); $(MAKE) $(MFLAGS)

$(LIB_VIZ_TOOL): force_look
	cd $(VIZ_TOOL); $(MAKE) $(MFLAGS)

$(LIB_MDP_METHODS): force_look
	cd $(MDP_METHODS); $(MAKE) $(MFLAGS)

$(LIB_INFO_METHODS): force_look
	cd $(INFO_METHODS); $(MAKE) $(MFLAGS)

$(LIB_ANIMATION): force_look
	cd $(ANIMATION); $(MAKE) $(MFLAGS)
	
$(LIB_HUNGARIAN): force_look
	cd $(HUNGARIAN); $(MAKE) $(MFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDE) -c $<

%.o: %.cc
	$(CC) $(CFLAGS) $(INCLUDE) -c $<


.PHONY: clean reallyclean
clean :
	$(ECHO) cleaning up in .
	-$(RM) -f $(EXE) $(OBJS) $(OBJLIBS)
	-$(RM) -f *.o *.a *.~ .*.swp
	-for d in $(DIRS); do (cd $$d; $(MAKE) clean ); done
reallyclean :
	$(ECHO) cleaning up in .
	-$(RM) -f $(EXE) $(OBJS) $(OBJLIBS)
	-$(RM) -f *.o *.a *.~ .*.swp
	-for d in $(DIRS); do (cd $$d; $(MAKE) reallyclean ); done

force_look :
	$(ECHO) force look:
	true

