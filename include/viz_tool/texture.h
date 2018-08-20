
#ifndef TEXTURE_H
#define TEXTURE_H

#ifdef __unix__
#include <GL/glut.h>
#endif

#ifdef __APPLE___
#include <OpenGl/gl.h>
#include <OpenGl/glu.h>
#include <GLUT/glut.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "bmploader.h"
#include "gldraw.h"

namespace viz_tool
{


  typedef struct {         // vertex coordinates - 3d and texture
      GLfloat x, y, z;     // 3d coords.
      GLfloat u, v;        // texture coords.
  } VERTEX;

  typedef struct {         // triangle
      VERTEX vertex[3];    // 3 vertices array
  } TRIANGLE;

  typedef struct {         // sector of a 3d environment
      int numtriangles;    // number of triangles in the sector
      TRIANGLE* triangle;  // pointer to array of triangles.
  } SECTOR;


   /**  functions **/

  // helper for SetupWorld.  reads a file into a string until a nonblank, non-comment line is found ("/" at the start indicating a comment); assumes lines < 255 characters long.
  void readstr(FILE *f, char *string);

  // loads the world from a text file.
  void loadConfigs(SECTOR& sector, char* filename);

  // Load Bitmaps And Convert To Textures
  GLvoid loadGLTextures(GLuint texture[][3]);

  // build the display list for a cube, e.g., the crate box.
  GLvoid cubeList(GLuint&);

  // build the display list for a plane, e.g., the wall
  GLvoid planeList(GLuint&, SECTOR& sec);

}

#endif


