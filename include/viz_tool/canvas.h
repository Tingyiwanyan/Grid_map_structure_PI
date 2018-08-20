
#ifndef CANVAS_H
#define CANVAS_H

#include "glm.h"
#include "gltb.h"
#include <cstdlib>
#include <unistd.h> //for usleep


namespace viz_tool
{

  const GLfloat light_position[] = { 30.0, 30.0, -30.0, 0.0 };

  extern GLdouble pan_x, pan_y, pan_z;

  void canvasInit(void);

  void reshape(int width, int height);

  void mouse(int button, int state, int x, int y);

  void motion(int x, int y);

  void keyboard(unsigned char key, int x, int y);

}

#endif

