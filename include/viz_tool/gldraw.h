/******************************************************************************
* File: gldraw.h
* Description: Some basic drawing
* Author: Lantao Liu
* Date: 6/2010 
******************************************************************************/

#ifndef GL_DRAW_H
#define GL_DRAW_H

#include "glfunc.h"
#include "math.h"
#include "assert.h"
#include "geometry_utils/Vector2.h"

using namespace geometry_utils;

// some basic components drawing.

namespace viz_tool
{

  //utility function for drawing basic components

  //2d x-y plane circle and ellipse, center is origin
  void drawCircle(float radius, int segments, bool plane=true);
  void drawEllipse(float major, float minor, int segments, bool plane);

  //3d ellipsoid
  void drawEllipsoid(GLfloat x, GLfloat y, GLfloat z);
  // the difference of two verisons lies in the orgin location 1: bottom, 2: middle
  void drawCylinder(float fTopR, float fBottomR, float fHeight);
  void drawCylinder2(int numStacks, int numSlices, float height, float radius);

  // representing goal/target
  void drawTargetSymbol(float radius, int segment);

  // in case gl linewidth does not work, use this. it's based on cylender
  void draw2DThickLine(const Vec2& a, const Vec2& b, double thickness);

  //draw 2d arrow starting from origin
  void draw2DArrow(float orient, float length, float linewidth);
  void draw2DArrow2(float x, float y, float linewidth); //x, y are end point

  //quadrotor components
  void drawBlade(void);
  void drawRotor(void);
  void drawMainRotor(void);
  void drawQuadrotor(bool spin=true);
  void drawEnergyBar(int);

  //simple ground vehicle components. fancy vehicles should load mesh models
  void drawWheel(void);
  void drawMobileRobot(bool spin=true);

  //marine glider
  void drawGlider(void);

  //draw and return x, y direction cell width
  pair<double, double>
  drawGridEnv(double _bx_pos, double _bx_neg,
	      double _by_pos, double _by_neg,
	      uint _x_divide, uint _y_divide);

  //init GL list for fast rendering
  void listInit();

}

#endif


