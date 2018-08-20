
#include "viz_tool/gldraw.h"

namespace viz_tool
{ 


//GLuint list_cube;
//GLuint list_plane;

GLuint list_rotor;
GLuint list_wheel;
GLuint list_mobile_robot;

GLfloat RotorSpinAngle=0;
GLfloat WheelSpinAngle=0;


void
drawCircle(float radius, int segments, bool plane)
{

  if(plane)
    glBegin( GL_TRIANGLE_FAN ); //plane
  else
    glBegin( GL_LINE_LOOP );    //loop
	float x=0, y=0; //origin
        for( int n = 0; n <= segments; ++n ) {
            float const t = 2*M_PI*(float)n/(float)segments;
            glVertex3f(x + sin(t)*radius, y + cos(t)*radius, 0);
        }
    glEnd();

}


void 
drawEllipse(float major, float minor, int segments, bool plane)
{

  if(plane)
    glBegin( GL_TRIANGLE_FAN ); //plane
  else
    glBegin( GL_LINE_LOOP );    //loop

        for( int n = 0; n <= segments; ++n ) {
            float const t = 2*M_PI*(float)n/(float)segments;
            glVertex3f(major*cos(t), minor*sin(t), 0);
        }
    glEnd();

}


void
drawTargetSymbol(float radius, int segment)
{

  //after calling the model, need to reset below 
  glEnable(GL_COLOR_MATERIAL);
  glPushMatrix();
    // the bottom disk layer, largest, red
    glColor3f(1, 0, 0);
    drawCircle(radius, segment);
    // the mid disk layer, white
    glColor3f(1, 1, 1);
    glTranslatef(0, 0, 0.007);
    drawCircle((2./3.)*radius, segment);
    // the top disk layer, smallest red
    glColor3f(1, 0, 0);
    glTranslatef(0, 0, 0.007);
    drawCircle((1./3.)*radius, segment);
  glPopMatrix();

}


void draw2DThickLine(const Vec2& a, const Vec2& b, double thickness){

  double orient = atan2(b.y()-a.y(), b.x()-a.x());
  double angle =orient*180/M_PI; 
  float length = (a-b).norm(); 

  glPushMatrix();
    glTranslatef(a.x(), a.y(), 0);
    glRotatef(90, 0, 1, 0);
    glRotatef(-angle, 1, 0, 0);
    drawCylinder(thickness, thickness, length); 
  glPopMatrix();

}


void draw2DArrow(float orient, float length,  float linewidth){

  //assume starting point (x1, y1) is origin
  float x1=0, y1=0; 
  float z = 0;

  //end point
  float x2, y2;
  x2 = x1 + length*cos(orient);
  y2 = y1 + length*sin(orient);

  //draw line
  glLineWidth(linewidth);
  glBegin(GL_LINES);
    glVertex3f(x1, y1, z);
    glVertex3f(x2, y2, z);
  glEnd();

  //triangle base and height
  double base   = 0.8;
  double height = 0.9;
  //scale wrt linewidth
  double scale = 0.4*linewidth;

  //if arrow length is almost shorter than arrow head, re-scale the head to be smaller
  if(height > 1.5*length)
    scale = length/height;

  float x_, y_; //a small offset that moves arrow forward, o/w it looks not sharp
  x_ = x2 + 0.1*height*cos(orient);
  y_ = y2 + 0.1*height*sin(orient);

  glPushMatrix();
    glTranslatef(x_, y_, z);
    glRotatef(orient*180/M_PI + 90, 0, 0, 1);
    //add the arrow at the end
    glBegin(GL_TRIANGLES); //Begin triangle coordinates
      glVertex3f(-0.5*base*scale, height*scale, z);
      glVertex3f(0.5*base*scale, height*scale, z);
      glVertex3f(0.0f, 0.0f, z);
    glEnd(); //End triangle coordinates
  glPopMatrix();

}


void draw2DArrow2(float x, float y, float linewidth){

  // ori wrt x and z plane (floor)
  double orient = atan2(y-0, x-0);
  //rotation angle in \pi
  double a =orient*180/M_PI + 90; 

  float z = 0.0; 

  //draw line
  glLineWidth(linewidth);
  glBegin(GL_LINES);
    glVertex3f(0, 0, z);
    glVertex3f(x, y, z);
  glEnd();

/*
  //triangle base and height
  double base   = 0.6;
  double height = 0.9;
  //scale wrt linewidth
  double scale  = 0.5*linewidth;

  //if arrow length is almost shorter than arrow head, re-scale the head to be smaller
  double length = sqrt(x*x + y*y);
  if(height > 1.5*length)
    scale = length/height;

  float x_, y_; //a small offset that moves arrow forward, o/w it looks not sharp
  x_ = x + 0.1*height*cos(orient);
  y_ = y + 0.1*height*sin(orient);

  glPushMatrix();
    glTranslatef(x_, y_, z);
    glRotatef(a, 0, 0, 1);
    //add the arrow at the end
    glBegin(GL_TRIANGLES); //Begin triangle coordinates
      glVertex3f(-0.5*base*scale, height*scale, z);
      glVertex3f(0.5*base*scale, height*scale, z);
      glVertex3f(0.0f, 0.0f, z);
    glEnd(); //End triangle coordinates
  glPopMatrix();
*/

}


void drawEllipsoid(GLfloat x, GLfloat y, GLfloat z)
{

  glPushMatrix();
  glScalef(x, y, z);
  glutSolidSphere(1.0, 20, 20);
  glPopMatrix();

}


// Used to generate a cylinder shape. the origin is at the bottom of the cylinder
void drawCylinder(float fTopR, float fBottomR, float fHeight)
{

  GLUquadricObj* pObj;
	  // To keep the original texture intact we need to set the current color to WHITE.

  pObj = gluNewQuadric();
	  // Creates a new quadrics object and returns a pointer to it.
  gluQuadricDrawStyle(pObj, GLU_FILL);

  //draw hull
  gluCylinder(pObj, fTopR, fBottomR, fHeight, 10, 1); //only one stack
  //draw the lids with a radius : fRadius.
  glPushMatrix();
    gluDisk(pObj, 0, fTopR, 10, 1 ); // bottom cover
    glTranslatef(0, 0, fHeight);
    gluDisk(pObj, 0, fTopR, 10, 1 ); // top cover
  glPopMatrix();

  gluDeleteQuadric(pObj);          // Free the Quadric

}


// the origin is at the middle of the cylinder
void drawCylinder2(int numStacks, int numSlices, float height, float radius)
{
  double majorStep = height / numStacks;
  double minorStep = 2.0 * M_PI / numSlices;
  int i, j;

  for (i = 0; i < numStacks; ++i) {
    GLfloat z0 = 0.5 * height - i * majorStep;
    GLfloat z1 = z0 - majorStep;

    glBegin(GL_TRIANGLE_STRIP);
    for (j = 0; j <= numSlices; ++j) {
      double a = j * minorStep;
      GLfloat x = radius * cos(a);
      GLfloat y = radius * sin(a);
      glNormal3f(x / radius, y / radius, 0.0);
      glTexCoord2f(j / (GLfloat) numSlices, i / (GLfloat) numStacks);
      glVertex3f(x, y, z0);

      glNormal3f(x / radius, y / radius, 0.0);
      glTexCoord2f(j / (GLfloat) numSlices, (i + 1) / (GLfloat) numStacks);
      glVertex3f(x, y, z1);
    }
    glEnd();
  }
}


void drawBlade()
{

  const GLfloat BladeWidth=0.1;
  const GLfloat BladeHeight=0.08;
  const GLfloat BladeLength=1.0;
  glPushMatrix();
    glTranslatef(0, BladeLength, 0);
    drawEllipsoid(BladeWidth, BladeLength, BladeHeight);
  glPopMatrix();

}


void drawRotor()
{

  drawBlade();

  glPushMatrix();
    drawBlade();
    glRotatef(120, 0, 0, 1);
    drawBlade();
    glRotatef(120, 0, 0, 1);
    drawBlade();
  glPopMatrix();

}


void drawMainRotor()
{

  const GLfloat MainRotorAxisHeight=0.6;
  const GLfloat MainRotorAxisRadius=0.05;

  glEnable(GL_COLOR_MATERIAL);
  glPushMatrix();
    glPushMatrix();
      //draw motor axis
      glColor3f(0.2, 0.2, 0.2);
      //glRotatef(-90, 1,0,0);
      drawCylinder(MainRotorAxisRadius, MainRotorAxisRadius, MainRotorAxisHeight);
      //draw motor
      glColor3f(0.8, 0.2, 0.2);
      drawCylinder(4*MainRotorAxisRadius, 4*MainRotorAxisRadius, 0.5*MainRotorAxisHeight);
    glPopMatrix();
    glTranslatef(0, 0, MainRotorAxisHeight);
    glColor3f(0, 0, 0);
    drawRotor();
  glPopMatrix();

}


void drawQuadrotor(bool spin)
{

  glPushMatrix();

    // draw frames
    glColor3f(0, 0, 0);
    glPushMatrix(); // beam 1
      glScalef(12, 1, 0.1);
      glutSolidCube(0.5);
    glPopMatrix();
    glPushMatrix(); //beam 2
      glScalef(1, 12, 0.1);
      glutSolidCube(0.5);
    glPopMatrix();
    glPushMatrix(); // the middle board
      glScalef(4, 4, 1);
      glutSolidCube(0.5);
    glPopMatrix();

    // draw 4 rotors
    glPushMatrix();
      glTranslatef(-3, 0, 0.025); //0.025 = (0.5 x 0.1)/2
      if(spin)
	glRotatef(RotorSpinAngle, 0,0,1);
      glCallList(list_rotor);
    glPopMatrix();
    glPushMatrix();
      glTranslatef(3, 0, 0.025);
      if(spin)
	glRotatef(RotorSpinAngle, 0,0,1);
      glCallList(list_rotor);
    glPopMatrix();
    glPushMatrix();
      glTranslatef(0, -3, 0.025);
      if(spin)
	glRotatef(-RotorSpinAngle, 0,0,1);
      glCallList(list_rotor);
    glPopMatrix();
    glPushMatrix();
      glTranslatef(0, 3, 0.025);
      if(spin)
	glRotatef(-RotorSpinAngle, 0,0,1);
      glCallList(list_rotor);
    glPopMatrix();

  glPopMatrix();

}


void drawGlider(void)
{

  //default orientation: (0, 1, 0)

  glPushMatrix();
    // draw body
    glPushMatrix();
      glColor3f(1.0, 1.0, 0.0);
      glRotatef(-90, 1.0, 0.0, 0.0);
      drawCylinder(0.32, 0.32, 2.4);
    glPopMatrix();

    //glider black head
    glPushMatrix();
      glColor3f(0.0, 0.0, 0.0);
      glTranslatef(0, 2.4, 0);
      drawEllipsoid(0.3, 0.6, 0.3);
    glPopMatrix();

    //glider oval tale
    glColor3f(1.0, 1.0, 0.0);
    glPushMatrix();
      glTranslatef(0, -0.2, 0);
      drawEllipsoid(0.3, 0.6, 0.3);
    glPopMatrix();

    //right wing
    glPushMatrix();
      glTranslatef(0.2, 0.1, 0);
      glBegin(GL_POLYGON);
	glVertex2f(0.0, 1.0);
	glVertex2f(0.0, 1.5);
	glVertex2f(1.2, 0.4);
	glVertex2f(1.2, 0.0);
      glEnd();
    glPopMatrix();

    //left wing
    glPushMatrix();
      glTranslatef(-0.2, 0.1, 0);
      glBegin(GL_POLYGON);
	glVertex2f(0.0, 1.0);
	glVertex2f(0.0, 1.5);
	glVertex2f(-1.2, 0.4);
	glVertex2f(-1.2, 0.0);
      glEnd();
    glPopMatrix();

  glPopMatrix();

}


void drawWheel(void){ 

  glutSolidTorus(0.3, 0.8 , 10, 20);

  // draw frames
  glRotatef(90, 0,1,0);
  drawCylinder(0.05, 0.05, 0.5);
  glRotatef(-180, 0,1,0);
  drawCylinder(0.05, 0.05, 0.5);
  glRotatef(-90, 1,0,0);
  drawCylinder(0.05, 0.05, 0.5);
  glRotatef(180, 1,0,0);
  drawCylinder(0.05, 0.05, 0.5);

}


//TODO this is drawn with Y as upward
void drawMobileRobot(bool spin){

  glPushMatrix();
    // draw body
    glColor3f(0.5, 0.5, 1);
    glPushMatrix();
      glScalef(12, 2, 6.8);
      //glScalef(6, 1, 3.4);
      //glRotatef(180, 1,0,0);
      glutSolidCube(0.5);
    glPopMatrix();

    // draw 4 wheels
    float wheel_offset = 2;
    glColor3f(0.3, 0.3, 0.3);
    glPushMatrix();
      glTranslatef(-wheel_offset, 0.025, -wheel_offset ); //0.025 = (0.5 x 0.1)/2
      if(spin)
	glRotatef(WheelSpinAngle, 0,0,1);
      glCallList(list_wheel);
    glPopMatrix();
    glPushMatrix();
      glTranslatef(wheel_offset, 0.025, -wheel_offset);
      if(spin)
	glRotatef(WheelSpinAngle, 0,0,1);
      glCallList(list_wheel);
    glPopMatrix();
    glPushMatrix();
      glTranslatef(-wheel_offset, 0.025, wheel_offset);
      if(spin)
	glRotatef(WheelSpinAngle, 0,0,1);
      glCallList(list_wheel);
    glPopMatrix();
    glPushMatrix();
      glTranslatef(wheel_offset, 0.025, wheel_offset);
      if(spin)
	glRotatef(WheelSpinAngle, 0,0,1);
      glCallList(list_wheel);
    glPopMatrix();

  glPopMatrix();

}


void listInit(){

  //cubeList(list_cube);
  //planeList(list_plane);

  list_rotor = glGenLists (1);
  glNewList(list_rotor, GL_COMPILE);
    drawMainRotor();
  glEndList();

  list_wheel = glGenLists (1);
  glNewList(list_wheel, GL_COMPILE);
    drawWheel();
  glEndList();

/*
  // only static rotors
  list_quadrotor = glGenLists (1);
  glNewList(list_quadrotor, GL_COMPILE);
    drawQuadrotor();
  glEndList();
*/

}


void drawEnergyBar(int level){

  const int numDiv = 10;
  const float width = 3;
  const float sep = 0.4;
  const float barHeight = 6/(float)numDiv;
  glTranslatef(-width/2, 0, 0);
  glBegin(GL_QUADS);
    glColor3f(0, 1, 0);
    for(float i = 0; i < level; i += (sep + barHeight)) {
      glVertex2f(0, i);
      glVertex2f(width, i);
      glVertex2f(width, i + barHeight);
      glVertex2f(0, i + barHeight);
    }
  glEnd();

}


pair<double, double>
drawGridEnv(double _bx_pos, double _bx_neg,
            double _by_pos, double _by_neg,
            uint _x_divide, uint _y_divide){

  double x_cell_width = (double)(_bx_pos - _bx_neg)/_x_divide;
  double y_cell_width = (double)(_by_pos - _by_neg)/_y_divide;

  // draw topo, all connection lines
  //glColor3f(0.0f, 0.6f, 1.0f);
  //glColor3f(0.6f, 0.8f, 1.0f);
  glColor3f(0.5f, 0.5f, 1.0f);
  glLineWidth(0.3);
  float z_pos = 0.02;

  //vertical
  glBegin(GL_LINES);
  for(uint i=0; i<=_x_divide; i++){
    glVertex3f(_bx_neg + i*x_cell_width, _by_neg, z_pos);
    glVertex3f(_bx_neg + i*x_cell_width, _by_pos, z_pos);
  }
  glEnd();
  //horizontal
  glBegin(GL_LINES);
  for(uint j=0; j<=_y_divide; j++){
    glVertex3f(_bx_neg, _by_neg + j*y_cell_width, z_pos);
    glVertex3f(_bx_pos, _by_neg + j*y_cell_width, z_pos);
  }
  glEnd();

  return pair<double, double>(x_cell_width, y_cell_width);

}


} //namespace

/*
void moveAgts(Agent* a, float velocity){

  assert(!a->waypoints.empty());
  Pose next_pose = a->waypoints.front();
  double orientation = atan2(next_pose.z-a->pose.z, next_pose.x-a->pose.x);
  //update the agent to face the moving direction
  //a->pose.a =180 -(orientation*180/M_PI - 90); //why this? atan?
  if(sqrt((a->pose.x-next_pose.x)*(a->pose.x-next_pose.x)+(a->pose.z-next_pose.z)*(a->pose.z-next_pose.z))>velocity ){
    a->pose.x += velocity*cos(orientation);
    a->pose.z += velocity*sin(orientation);
  }
  else{
    //delete this waypoint
    a->waypoints.pop_front();
    //update next pose
    if(!a->waypoints.empty())
      next_pose = a->waypoints.front();
  }

}

void moveAgts(Agent& a, float velocity, float orientation, Pose goal){

  if(sqrt((a.pose.x-goal.x)*(a.pose.x-goal.x)+(a.pose.z-goal.z)*(a.pose.z-goal.z))>velocity ){
    a.pose.x += velocity*cos(orientation);
    a.pose.z += velocity*sin(orientation);
  }
  else{
    //delete this waypoint
    a.waypoints.pop_front();
  }

}
*/


