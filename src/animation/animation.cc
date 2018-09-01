
#include "fluid/fluid2d.h"
#include "animation/animation.h"

// ascii codes for various special keys
#define ESCAPE 27

//keyboard control of camera
GLfloat Xrot;   // x rotation
GLfloat Yrot;   // y rotation
GLfloat Ztran=0.0f; // translation/depth into the screen.

int BackColor=0;

void
display(void)
{
//std::cout<<"Im here in display"<<std::endl;
  //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  glTranslatef(0.0f,0.0f,Ztran);        // move z units out from the screen.
  glRotatef(Xrot,1.0f,0.0f,0.0f);       // Rotate On The X Axis
  glRotatef(Yrot,0.0f,1.0f,0.0f);       // Rotate On The Y Axis

  // sky color, dark sky needs a better translucent floor
  //glClearColor(0, 0, 0, 1.0); // black
  glClearColor(BackColor, BackColor, BackColor, 1.0);  // white

  //Tell GL new light source position to create shading, for darker shadow!
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  // run clock (the whole program only need run it once, it's a static class)
  utils::WallClock::runWallClock();

  //shadowMatrix(floorShadow, floorPlane, light_position);

  glPushMatrix();

    gltbMatrix(); 			//for camera rotation
    glTranslatef(pan_x, pan_y, pan_z); 	//for camera translation

    render_mdp::render();
    //fluid::draw_velocity();

  glPopMatrix();

  //keyboard special keys reset, so key pressing here is pulses
  Xrot=0;
  Yrot=0;
  Ztran=0;

  glutSwapBuffers();

}


void
normalKeyPressed(unsigned char key, int x, int y){

  // avoid thrashing this procedure
  std::cout<<"Im here in normalkeypress"<<std::endl;
  usleep(100);

  render_mdp::key(key, x, y);

  fluid::key_func(key, x, y);

  switch (key) {
    case 'b':
    case 'B':
      if(BackColor) BackColor = 0;
      else BackColor = 1;
      break;
    case ESCAPE: // kill everything.
    case 'q':
    case 'Q':
      fluid::free_data();
      //DestroyAll();
      exit(1);
    default:
      printf ("Key %d pressed. No action there yet.\n", key);
      break;
  }

}


void
specialKeyPressed(int key, int x, int y)
{

  /* avoid thrashing this procedure */
  usleep(100);

  switch (key) {
  case GLUT_KEY_PAGE_UP: // move the cube into the distance.
      Ztran-=0.2f;
      break;
  case GLUT_KEY_PAGE_DOWN: // move the cube closer.
      Ztran+=0.2f;
      break;
  case GLUT_KEY_UP: // decrease x rotation speed;
      Xrot-=0.5f;
      break;
  case GLUT_KEY_DOWN: // increase x rotation speed;
      Xrot+=0.5f;
      break;
  case GLUT_KEY_LEFT: // decrease y rotation speed;
      Yrot-=0.5f;
      break;
  case GLUT_KEY_RIGHT: // increase y rotation speed;
      Yrot+=0.5f;
      break;
  default:
      break;
  }

}


void
idle(void)
{
    display();
    glutPostRedisplay();
}


int
animation(int argc, char** argv)
{

  glutInitWindowSize(1024, 1024);
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STENCIL | GLUT_MULTISAMPLE);
  glutCreateWindow("Simu");

  canvasInit();
  listInit();
  //textureInit(sec, texture2);

  //three high level planning methods
  //methodManager();

  render_mdp::init();		//must be after mdp initialized
  fluid::init();
  //initMDP();

  glutReshapeFunc(&reshape);
  glutDisplayFunc(&display);

  glutKeyboardFunc(&normalKeyPressed);
  glutSpecialFunc(&specialKeyPressed);
  //glutMouseFunc(&fluid::mouse_func);
  glutMouseFunc(&mouse);
  //glutMotionFunc(&fluid::motion_func);
  glutMotionFunc(&motion);
  glutIdleFunc(&idle);


  //modelInit();
  //makeCheckerFloorTexture();
  //makeCircleFloorTexture();

  /* Setup floor plane for projected shadow calculations. */
  render_mdp::findPlaneWrapper();

  glutMainLoop();

  return 0;

}
