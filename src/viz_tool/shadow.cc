
#include "viz_tool/shadow.h"

namespace viz_tool
{


enum { X, Y, Z, W };
enum { A, B, C, D };

/* Create a matrix that will project the desired shadow. */
void
shadowMatrix(GLfloat shadowMat[4][4],
	     GLfloat groundplane[4],
  	     const GLfloat lightpos[4])
{

  GLfloat dot;

  /* Find dot product between light position vector and ground plane normal. */
  dot = groundplane[X] * lightpos[X] +
    groundplane[Y] * lightpos[Y] +
    groundplane[Z] * lightpos[Z] +
    groundplane[W] * lightpos[W];

  shadowMat[0][0] = dot - lightpos[X] * groundplane[X];
  shadowMat[1][0] = 0.f - lightpos[X] * groundplane[Y];
  shadowMat[2][0] = 0.f - lightpos[X] * groundplane[Z];
  shadowMat[3][0] = 0.f - lightpos[X] * groundplane[W];

  shadowMat[X][1] = 0.f - lightpos[Y] * groundplane[X];
  shadowMat[1][1] = dot - lightpos[Y] * groundplane[Y];
  shadowMat[2][1] = 0.f - lightpos[Y] * groundplane[Z];
  shadowMat[3][1] = 0.f - lightpos[Y] * groundplane[W];

  shadowMat[X][2] = 0.f - lightpos[Z] * groundplane[X];
  shadowMat[1][2] = 0.f - lightpos[Z] * groundplane[Y];
  shadowMat[2][2] = dot - lightpos[Z] * groundplane[Z];
  shadowMat[3][2] = 0.f - lightpos[Z] * groundplane[W];

  shadowMat[X][3] = 0.f - lightpos[W] * groundplane[X];
  shadowMat[1][3] = 0.f - lightpos[W] * groundplane[Y];
  shadowMat[2][3] = 0.f - lightpos[W] * groundplane[Z];
  shadowMat[3][3] = dot - lightpos[W] * groundplane[W];

}


/* Find the plane equation given 3 points. */
void
findPlane(GLfloat plane[4], GLfloat v0[3], GLfloat v1[3], GLfloat v2[3])
{

  GLfloat vec0[3], vec1[3];

  /* Need 2 vectors to find cross product. */
  vec0[X] = v1[X] - v0[X];
  vec0[Y] = v1[Y] - v0[Y];
  vec0[Z] = v1[Z] - v0[Z];
  
  vec1[X] = v2[X] - v0[X];
  vec1[Y] = v2[Y] - v0[Y];
  vec1[Z] = v2[Z] - v0[Z];

  /* find cross product to get A, B, and C of plane equation */
  plane[A] = vec0[Y] * vec1[Z] - vec0[Z] * vec1[Y];
  plane[B] = -(vec0[X] * vec1[Z] - vec0[Z] * vec1[X]);
  plane[C] = vec0[X] * vec1[Y] - vec0[Y] * vec1[X];

  plane[D] = -(plane[A] * v0[X] + plane[B] * v0[Y] + plane[C] * v0[Z]);
}


// draw the shadow for the drawing routine of "drawAll"
void drawShadow(GLfloat floorShadow[4][4], void (*drawAll)() ){

  /* Render 50% black shadow color on top of whatever the
     floor appareance is. */
#ifdef GL_EXT_polygon_offset 
	glEnable (GL_POLYGON_OFFSET_EXT); 
	//glPolygonOffsetEXT (1., 1./(float)0x10000); 
#endif /* GL_EXT_polygon_offset */

  //glEnable(GL_POLYGON_OFFSET_EXT);
  glEnable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_LIGHTING);  /* Force the 50% black. */
  glColor4f(0.0, 0.0, 0.0, 0.5);

  glPushMatrix();
  /* Project the shadow. */
  glMultMatrixf((GLfloat *) floorShadow);
  drawAll();
  glPopMatrix();

#ifdef GL_EXT_polygon_offset 
  glDisable(GL_POLYGON_OFFSET_EXT);
#endif /* GL_EXT_polygon_offset */
  //glDisable(GL_POLYGON_OFFSET_EXT);

  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);
  glEnable(GL_LIGHTING);
}


// draw the Reflection, the GLfloor.h need be included to use "drawFloor"
void drawReflection(const GLfloat light_position[], GLfloat floorVertices[4][3], void (*drawAll)()){

/* chunk below hides the stencil reflection */
  glEnable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  // Draw 1 into the stencil buffer. 
  glEnable(GL_STENCIL_TEST);
  glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);
  glStencilFunc(GL_ALWAYS, 1, 0xffffffff);
  // Now render floor; floor pixels just get their stencil set to 1. 
  drawFloor(floorVertices, 8);
  // Re-enable update of color and depth. 
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glEnable(GL_DEPTH_TEST);
  // Now, only render where stencil is set to 1. 
  glStencilFunc(GL_EQUAL, 1, 0xffffffff);  // draw if ==1 
  glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
/* chunk above hides the stencil reflection */


  glPushMatrix();
    // The critical reflection step: Reflect through the floor (the Y=0 plane) to make a relection. 
    //glScalef(1.0, -1.0, 1.0);  // y upward
    glScalef(1.0, 1.0, -1.0);   // z upward
    // Reflect the light position.
    //glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    // To avoid our normals getting reversed and hence botched lighting on the reflection, turn on normalize.  
    glEnable(GL_NORMALIZE);
    // hmmmm, fliping GL_BACK/FRONT does the reflection tricks 1/25/2015!!
    glCullFace(GL_BACK);
    drawAll();
    // Disable noramlize again and re-enable back face culling. 
    glDisable(GL_NORMALIZE);
    glCullFace(GL_FRONT);
  glPopMatrix();

  // Switch back to the unreflected light position. 
  //glLightfv(GL_LIGHT0, GL_POSITION, light_position);

  glDisable(GL_STENCIL_TEST);
  glDisable(GL_CULL_FACE);

  // Draw "top" of floor.  Use blending to blend in reflection. 
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor4f(0.9, 1.0, 1.0, 0.5);
  drawFloor(floorVertices, 16);
  glDisable(GL_BLEND);

}


} //namespace

