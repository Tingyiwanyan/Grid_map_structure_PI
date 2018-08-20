
#ifndef TEX_WALLS_H
#define TEX_WALLS_H

#include "texture.h"

namespace viz_tool
{

  void textureInit(SECTOR& sec, GLuint texture[][3]);

  /* The main drawing functions. */
  GLvoid drawAttenuateWalls(GLuint texture[][3]);

  GLvoid drawFenceWalls(GLuint texture[][3]);
  GLvoid draw_T_Walls(GLuint texture[][3]);
  GLvoid draw_L_Walls(GLuint texture[][3]);
  GLvoid draw_XbyY_block(uint x, uint y, GLuint texture[][3]);

  GLvoid drawSurveillanceWalls(GLuint texture[][3]);
  GLvoid drawExplorationWalls(GLuint texture[][3]);
  GLvoid drawHorizonWalls(GLuint texture[][3]);

}

#endif


