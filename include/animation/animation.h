
#ifndef ANIMATION_H
#define ANIMATION_H

#include "utils/clock.h"
#include "render_mdp.h"

using namespace mdp_planner;
using namespace viz_tool;

//for view/camera position
extern GLdouble viz_tool::pan_x, 
		viz_tool::pan_y, 
		viz_tool::pan_z;


void display(void);

void idle(void);

int animation(int argc, char** argv);



#endif


