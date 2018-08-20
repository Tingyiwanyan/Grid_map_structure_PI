
#include "viz_tool/canvas.h"

namespace viz_tool
{


// center of the plot, will be moved using middle button
GLdouble   pan_x = 0.0;
GLdouble   pan_y = 0.0;
GLdouble   pan_z = 0.0;


void
canvasInit(void)
{

  //gltb camera-mouse control init
  gltbInit(GLUT_LEFT_BUTTON);
  //gltbAnimate(GL_TRUE);	//see gltb.h for usage
  gltbAnimate(GL_FALSE);

  GLfloat mat_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
  GLfloat mat_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
  GLfloat mat_specular[] = { 0.0, 0.0, 0.0, 1.0 };
  GLfloat mat_shininess = 65.0;

  GLfloat light_ambient[] = { 0.8, 0.8, 0.8, 1.0 };
  GLfloat light_diffuse[] = { 0.9, 0.9, 0.9, 1.0 };
  GLfloat light_specular[] = { 0.8, 0.8, 0.8, 1.0 };

  glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

  //light is default (natural) if globally no position is specified
  //glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  //below should be off, it affects the reflection
  //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

  glEnable(GL_DEPTH_TEST);
  //glEnable(GL_TEXTURE_2D); //turn it on only when needed and off after use

  // flicking color occurs for overlapped planes (different colors)
  // for non-flicking shadow, but must enable GL_POLYGON_OFFSET_EXT to use it 
  glPolygonOffset(-0.1, -0.002);
  // also CULL_FACE should be (later) turned on
  //glEnable(GL_CULL_FACE); 	//remove back face

}


void
reshape(int width, int height)
{

  gltbReshape(width, height);

  glViewport(0, 0, width, height);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60.0, (GLfloat)height / (GLfloat)width, 1.0, 128.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  //farther view
  //gluLookAt(0.0, 65, 40, / camera pos, z must be >0, otherwise inside floor
  gluLookAt(0.0, -10, 50, //camera pos, y cannot be >0, otherwise inside floor
  0.0, 0.0, 0.0,      // center is moved along y+, s.t. the floor is lowered
  0.0, 1.0, 0.0);      // camera up is in +Y direction (if set +Z direction, the sceen is fliped upside down around y=+/-0 )


/*
  //nearer view
  gluLookAt(0.0, 15.0, 22.0,  // camera pos
  0.0, -5, 0.0,      // center is moved along y, s.t. the floor is lowered
  0.0, 1.0, 0.0);      // up is in postivie Y direction
*/

}


static GLint      mouse_state;
static GLint      mouse_button;
static GLdouble	  height = 100;	//tuned.. Pan function needs to be improved/replaced !!!


void
mouse(int button, int state, int x, int y)
{

  GLdouble model[4*4];
  GLdouble proj[4*4];
  GLint view[4];

  /* fix for two-button mice -- left mouse + shift = middle mouse */
  if (button == GLUT_LEFT_BUTTON && glutGetModifiers() & GLUT_ACTIVE_SHIFT)
      button = GLUT_MIDDLE_BUTTON;

  gltbMouse(button, state, x, y);

  mouse_state = state;
  mouse_button = button;

  if (state == GLUT_DOWN && button == GLUT_MIDDLE_BUTTON) {
      glGetDoublev(GL_MODELVIEW_MATRIX, model);
      glGetDoublev(GL_PROJECTION_MATRIX, proj);
      glGetIntegerv(GL_VIEWPORT, view);
      gluProject((GLdouble)x, (GLdouble)y, height,
	  model, proj, view,
	  &pan_x, &pan_y, &pan_z);
      gluUnProject((GLdouble)x, (GLdouble)y, pan_z,
	  model, proj, view,
	  &pan_x, &pan_y, &pan_z);
      pan_y = -pan_y;
  }

  glutPostRedisplay();

}


void
motion(int x, int y)
{

  GLdouble model[4*4];
  GLdouble proj[4*4];
  GLint view[4];

  gltbMotion(x, y);

  if (mouse_state == GLUT_DOWN && mouse_button == GLUT_MIDDLE_BUTTON) {
      glGetDoublev(GL_MODELVIEW_MATRIX, model);
      glGetDoublev(GL_PROJECTION_MATRIX, proj);
      glGetIntegerv(GL_VIEWPORT, view);
      gluProject((GLdouble)x, (GLdouble)y, height,
	  model, proj, view,
	  &pan_x, &pan_y, &pan_z);
      gluUnProject((GLdouble)x, (GLdouble)y, pan_z,
	  model, proj, view,
	  &pan_x, &pan_y, &pan_z);
      pan_y = -pan_y;
  }

  glutPostRedisplay();

}



}//namespace


