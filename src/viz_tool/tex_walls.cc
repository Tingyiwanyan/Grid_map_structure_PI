
#include "viz_tool/tex_walls.h"

namespace viz_tool
{


GLuint list_cube;
GLuint list_plane;
// types of textures
enum textures {brick, crate};


void textureInit(SECTOR& sec, GLuint texture[][3]){
  loadConfigs(sec, (char*)"configs/wall.txt");
  loadGLTextures(texture);  
  cubeList(list_cube);		//passed the value into list_cube
  planeList(list_plane, sec);	//passed the value into list_plane
}


GLvoid drawAttenuateWalls(GLuint texture[][3]){

  enum {filter0, filter1, filter2}; //filter2 is most natrual
  glEnable(GL_TEXTURE_2D); 

   //draw walls, using cubes with brick texture
    glBindTexture(GL_TEXTURE_2D, texture[crate][filter2]);  // pick the texture.
    const float cube_width = 3; // adjusted in cubeList() function
    for(int i=0; i<10; i++){
      glPushMatrix();
	// the cube has width cube_width for each size
        glTranslatef((i+2)*cube_width, cube_width/2, 0);
        glCallList(list_cube);
      glPopMatrix();
      glPushMatrix();
	// the cube has width cube_width for each size
        glTranslatef(-(i+2)*cube_width, cube_width/2, 0);
        glCallList(list_cube);
      glPopMatrix();
    }

  glDisable(GL_TEXTURE_2D); 

}


GLvoid drawFenceWalls(GLuint texture[][3]){

  enum {filter0, filter1, filter2}; //filter2 is most smooth
  glEnable(GL_TEXTURE_2D);

   GLfloat bound_x = 30; //BOUND_X*1.5;
   GLfloat bound_z = 30; //BOUND_Z*1.5;

   //draw walls, using cubes with brick texture
    glBindTexture(GL_TEXTURE_2D, texture[brick][filter1]);  // pick the texture.
    const float cube_width = 3;
    for(int i=0; i<20; i++){
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(i*cube_width-bound_x, cube_width/2, bound_z-cube_width);
        glCallList(list_cube);
      glPopMatrix();
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(i*cube_width-bound_x, cube_width/2, -bound_z);
        glCallList(list_cube);
      glPopMatrix();
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(-bound_x, cube_width/2, i*cube_width -bound_z);
        glCallList(list_cube);
      glPopMatrix();
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(bound_x, cube_width/2, i*cube_width -bound_z);
        glCallList(list_cube);
      glPopMatrix();
    }

  glDisable(GL_TEXTURE_2D);

}


GLvoid draw_T_Walls(GLuint texture[][3]){

  enum {filter0, filter1, filter2}; //filter2 is most smooth
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture[brick][filter1]);  // pick the texture.

  const float cube_width = 3;
  for(int i=0; i<5; i++){
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(i*cube_width-2*cube_width, cube_width/2, 0);
        glCallList(list_cube);
      glPopMatrix();
  }

  for(int i=0; i<3; i++){
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(0, cube_width/2, (i+1)*cube_width);
        glCallList(list_cube);
      glPopMatrix();
  }
  glDisable(GL_TEXTURE_2D);

}


GLvoid draw_XbyY_block(uint x, uint y, GLuint texture[][3]){

  enum {filter0, filter1, filter2}; //filter2 is most smooth
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture[brick][filter1]);  // pick the texture.

  const float cube_width = 3;
  for(int i=0; i<x; i++){
    for(int j=0; j<y; j++){
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(0, cube_width/2, 0);
        glTranslatef(i*cube_width, 0, j*cube_width);
        glCallList(list_cube);
      glPopMatrix();
     }//j
  }//i
/*
  for(int i=0; i<x; i++){
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(cube_width/2, cube_width/2, (i+1)*cube_width);
        glCallList(list_cube);
      glPopMatrix();
  }
*/
  glDisable(GL_TEXTURE_2D);

}


GLvoid draw_L_Walls(GLuint texture[][3]){

  enum {filter0, filter1, filter2}; //filter2 is most smooth
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture[brick][filter1]);  // pick the texture.

  const float cube_width = 3;

  for(int i=0; i<4; i++){
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(0, cube_width/2, (i+1)*cube_width);
        glCallList(list_cube);
      glPopMatrix();
  }

  for(int i=0; i<4; i++){
      glPushMatrix();
        // the cube has width cube_width for each size
        glTranslatef(i*cube_width, cube_width/2, 0);
        glCallList(list_cube);
      glPopMatrix();
  }
  glDisable(GL_TEXTURE_2D);

}


GLvoid drawSurveillanceWalls(GLuint texture[][3]){

  glEnable(GL_TEXTURE_2D);
  drawFenceWalls(texture);
  
  glPushMatrix();
    glTranslatef(-12, 0, -11);
    glRotatef(90, 0, 1, 0);
    draw_T_Walls(texture);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(0, 0, 15);
    glRotatef(180, 0, 1, 0);
    draw_T_Walls(texture);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(18, 0, -10);
    //glRotatef(90, 0, 1, 0);
    draw_L_Walls(texture);
  glPopMatrix();

  glDisable(GL_TEXTURE_2D);

}


GLvoid drawExplorationWalls(GLuint texture[][3]){

  glEnable(GL_TEXTURE_2D);
  drawFenceWalls(texture);

  //draw three 3x2 blocks, locate on top
  for(int i=-1; i<2; i++){
  glPushMatrix();
    glTranslatef(-3, 0, 0);
    glTranslatef(15*i, 0, -30);
    draw_XbyY_block(3,2, texture);
  glPopMatrix();
  }

  //draw three 5x2 blocks, locate on left
  for(int i=0; i<2; i++){
  glPushMatrix();
    glTranslatef(0, 0, -5);
    glTranslatef(-30, 0, 14*i-7);
    draw_XbyY_block(2, 5, texture);
  glPopMatrix();
  }

  //draw three 5x2 blocks, locate on right
  for(int i=0; i<2; i++){
  glPushMatrix();
    glTranslatef(30-3*5, 0, 14*i-7);
    draw_XbyY_block(2, 5, texture);
  glPopMatrix();
  }

/*
  //draw three 2x2 blocks, locate in middle
  for(int i=0; i<2; i++){
  glPushMatrix();
    glTranslatef(0, 0, 14*i-7);
    draw_XbyY_block(2,2, texture);
  glPopMatrix();
  }
*/

  glPushMatrix();
    glTranslatef(-5, 0, -15);
    draw_XbyY_block(5,3, texture);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(-5, 0, 6);
    draw_XbyY_block(4,3, texture);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(-21, 0, 18);
    draw_XbyY_block(2,3, texture);
  glPopMatrix();

  glDisable(GL_TEXTURE_2D);

}


GLvoid drawHorizonWalls(GLuint texture[][3]){

  drawFenceWalls(texture);

  glPushMatrix();
    glTranslatef(-17, 0, -14);
    //glTranslatef(-17, 0, -17);
    // each cube has width 3, 4x4 cubes
    draw_XbyY_block(4,5, texture);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(-14, 0, 8);
    //glTranslatef(-17, 0, 5);
    draw_XbyY_block(4,4, texture);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(12, 0, -12);
    draw_XbyY_block(3,3, texture);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(12, 0, 8);
    draw_XbyY_block(3,4, texture);
  glPopMatrix();

}


}//namespace
