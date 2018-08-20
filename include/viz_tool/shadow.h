
#ifndef GL_SHADOW_H
#define GL_SHADOW_H

#include <stdlib.h>
#include "floor.h"


namespace viz_tool
{

  /* Create a matrix that will project the desired shadow. */
  void
  shadowMatrix(GLfloat shadowMat[4][4],
	      GLfloat groundplane[4],
	      const GLfloat lightpos[4]);

  /* Find the plane equation given 3 points. */
  void
  findPlane(GLfloat plane[4], GLfloat v0[3], GLfloat v1[3], GLfloat v2[3]);

  /* draw the shadow for the drawing routine of "drawAll" */
  void 
  drawShadow(GLfloat floorShadow[4][4], void (*drawAll)() );

  /* draw the Reflection, the GLfloor.h need be included to use "drawFloor" */
  void 
  drawReflection(const GLfloat light_position[], 
		 GLfloat floorVertices[4][3], 
		 void (*drawAll)());

}

#endif


