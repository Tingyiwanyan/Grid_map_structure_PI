
/******************************************************************************
* File: GLfunc.h
* Description: Some basic drawing functions for drawing.
* Author: Lantao Liu
* Date: 6/2010 
******************************************************************************/

#ifndef GLFUNC_H
#define GLFUNC_H

#ifdef __unix__
#include <GL/glut.h>
#endif

#ifdef __APPLE___
#include <OpenGl/gl.h>
#include <OpenGl/glu.h>
#include <GLUT/glut.h>
#endif

#include <vector>
#include <string.h>
#include "geometry_utils/Vector3.h"
#include "viz_tool/canvas.h"

using namespace std;
using namespace geometry_utils;


namespace viz_tool
{


  typedef struct {
    size_t r; size_t g;  size_t b;
  } RGB;

  // store the fonts:
  extern void* glutFonts[7];


  // compute normal for a plane (triangle)
  Vec3f computeSurfaceNormal(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3);

  // the text print/draw function
  void glutPrint(float x, float y, void* font, char* text, 
		 float r, float g, float b, float a);

  // @ _num: the number of RGBs in the vector
  // @ _color: the atmosphere of the color, e.g. 'r': red mainly. can be 'g','b',''. If '', then grey atmosphere; 
  vector<RGB> generateRGB(uint _num, char _color);

  //set color in colormap style
  // @lambda [0, 1], lambda > 1 will be looped back to [0, 1]
  void colormap(double lambda);
  
  //used in colormap
  void pSetHSV(float h, float s, float v);

  void modelInit(void);

}//namespace

#endif


