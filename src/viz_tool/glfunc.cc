
#include "viz_tool/glfunc.h"

namespace viz_tool
{


GLMmodel*  model_robot;
GLMmodel*  model_cone;
GLuint model_list_robot, model_list_cone;

GLfloat smoothing_angle = 90.0;
GLfloat orig_scale;

char* Model_File = (char*)"data/mesh_objs/create_model.obj";



Vec3f computeSurfaceNormal(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3) {

  Vec3f v12 = p2;
  v12 -= p1;
  Vec3f v23 = p3;
  v23 -= p2;
  Vec3f normal = v12.cross(v23);
  normal.normalize();

  return normal;

}


/*
// find it in geometry_utils/Quaternion.h
void 
Quaternion2AxisAngle(const Vec4f& quat, Vec3f& axis, float *angle)
{
  double x, y, z;
  x=quat.x(); y=quat.y(); z=quat.z();
  float scale = sqrt(x * x + y * y + z * z);
  axis->x = x / scale;
  axis->y = y / scale;
  axis->z = z / scale;
  &angle = acos(w) * 2.0f; //w must be in [-1, 1]
}
*/


// Here are the fonts:
void* glutFonts[7] = { 
  GLUT_BITMAP_9_BY_15,
  GLUT_BITMAP_8_BY_13,
  GLUT_BITMAP_TIMES_ROMAN_10,
  GLUT_BITMAP_TIMES_ROMAN_24,
  GLUT_BITMAP_HELVETICA_10,
  GLUT_BITMAP_HELVETICA_12,
  GLUT_BITMAP_HELVETICA_18
};


void glutPrint(float x, float y, void* font, char* text, float r, float g, float b, float a)
{
  if(!text || !strlen(text)) return;
  bool blending = false;
  if(glIsEnabled(GL_BLEND)) blending = true;
  glEnable(GL_BLEND);
  glColor4f(r,g,b,a);
  glRasterPos2f(x,y);
  while (*text) {
      glutBitmapCharacter(font, *text);
      //glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *text);
      text++;
  }
  if(!blending) glDisable(GL_BLEND);
}


vector<RGB> generateRGB(uint _num, char _color){

  vector<RGB> rgbs;

  RGB rgb;
  switch (_color) {
    case 'r':
      for(uint i=0; i<_num; i++){
	rgb.r = 127 + 128*drand48();
	rgb.g = 128*drand48();
	rgb.b = 128*drand48();
	rgbs.push_back(rgb);
      }
      break;
    case 'g':
      for(uint i=0; i<_num; i++){
	rgb.r = 128*drand48();
	rgb.g = 127 + 128*drand48();
	rgb.b = 128*drand48();
	rgbs.push_back(rgb);
      }
      break;
    case 'b':
      for(uint i=0; i<_num; i++){
	rgb.r = 128*drand48();
	rgb.g = 128*drand48();
	rgb.b = 127 + 128*drand48();
	rgbs.push_back(rgb);
      }
      break;
    case 'a':   // uniformly green color (for agents)
      for(uint i=0; i<_num; i++){
	rgb.r = 0;
	rgb.g = 0.85 * 255;
	rgb.b = 0;
	rgbs.push_back(rgb);
      }
      break;
    case 't':   // uniformly blue color (for tasks)
      for(uint i=0; i<_num; i++){
	rgb.r = 0.4 * 255;
	rgb.g = 0.58 * 255;
	rgb.b = 0.93 * 255;
	rgbs.push_back(rgb);
      }
      break;
    case NULL:
      for(uint i=0; i<_num; i++){
	size_t c = 255*drand48();
	rgb.r = rgb.g = rgb.b = c;
	rgbs.push_back(rgb);
      }
      break;
    default: 
      for(uint i=0; i<_num; i++){
	rgb.r = 255*drand48();
	rgb.g = 255*drand48();
	rgb.b = 255*drand48();
	rgbs.push_back(rgb);
      }
      break;
  }

  return rgbs;
}


void colormap(double lambda){

  //assert(lambda >= 0 && lambda <=1);
  
  pSetHSV(360*lambda, 1, 1);
  
}


//http://forum.openframeworks.cc/t/hsv-color-setting/770
void pSetHSV(float h, float s, float v) 
{
    // H [0, 360] S and V [0.0, 1.0].
    int i = (int)floor(h / 60.0f) % 6;
    float f = h / 60.0f - floor(h / 60.0f);
    float p = v * (float)(1 - s);
    float q = v * (float)(1 - s * f);
    float t = v * (float)(1 - (1 - f) * s);
    switch (i) 
    {
        case 0: glColor3f(v, t, p);
            break;
        case 1: glColor3f(q, v, p);
            break;
        case 2: glColor3f(p, v, t);
            break;
        case 3: glColor3f(p, q, v);
            break;
        case 4: glColor3f(t, p, v);
            break;
        case 5: glColor3f(v, p, q);
    }
}   


void modelInit(void){

  if(!model_robot){
    model_robot = glmReadOBJ(Model_File);
    printf("loaded model file: %s.\n", Model_File);
    assert(model_robot);
    orig_scale = glmUnitize(model_robot);
    glmFacetNormals(model_robot);
    glmVertexNormals(model_robot, smoothing_angle);
  }
  if(!model_cone){
    model_cone = glmReadOBJ((char*)"objs/cone.obj");
    printf("loaded model file: %s.\n", (char*)"objs/cone.obj");
    assert(model_cone);
    orig_scale = glmUnitize(model_cone);
    glmFacetNormals(model_cone);
    glmVertexNormals(model_cone, smoothing_angle);
  }

  double scale = 1;
  glmScale(model_robot, scale);
  glmScale(model_cone, scale);
  // list is much recommended to enhance performance
  if (model_list_robot)
      glDeleteLists(model_list_robot, 1);
  if (model_list_cone)
      glDeleteLists(model_list_cone, 1);

  // flat is not auto processed, smooth has adjusted normals
  //model_list_robot = glmList(model_robot, GLM_FLAT | GLM_MATERIAL);
  model_list_robot = glmList(model_robot, GLM_SMOOTH | GLM_MATERIAL);
  model_list_cone = glmList(model_cone, GLM_SMOOTH | GLM_MATERIAL);

}


}//namespace

