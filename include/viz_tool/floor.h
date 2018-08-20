
#ifndef GL_FLOOR_H
#define GL_FLOOR_H

#include <stdlib.h>
#include <stdio.h>

#ifdef __unix__
#include <GL/glut.h>
#endif

#ifdef __APPLE___
#include <OpenGl/gl.h>
#include <OpenGl/glu.h>
#include <GLUT/glut.h>
#endif

namespace viz_tool
{

  void
  makeCheckerFloorTexture(bool useMipmaps=false, bool linearFiltering=false );

  void
  makeCircleFloorTexture(bool useMipmaps=false, bool linearFiltering=false );

  /* Draw a floor (possibly textured). */
  void
  drawFloor(GLfloat floorVertices[4][3], 
	  GLfloat tex_scale=4, bool useTexture=true);

}

#endif


