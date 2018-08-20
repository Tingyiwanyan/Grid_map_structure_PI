
#include "viz_tool/floor.h"

namespace viz_tool
{


/* Nice floor texture tiling pattern. */
static char *circles[] = {
(char*)  "....xxxx........",
(char*)  "..xxxxxxxx......",
(char*)  ".xxxxxxxxxx.....",
(char*)  ".xxx....xxx.....",
(char*)  "xxx......xxx....",
(char*)  "xxx......xxx....",
(char*)  "xxx......xxx....",
(char*)  "xxx......xxx....",
(char*)  ".xxx....xxx.....",
(char*)  ".xxxxxxxxxx.....",
(char*)  "..xxxxxxxx......",
(char*)  "....xxxx........",
(char*)  "................",
(char*)  "................",
(char*)  "................",
(char*)  "................",
};


//white-grey checker
GLubyte checker[] = {
    0xE8, 0xE8, 0xE8, 0xff, 0xff, 0xff, 0xE8, 0xE8, 0xE8, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xE8, 0xE8, 0xE8, 0xff, 0xff, 0xff, 0xE8, 0xE8, 0xE8,
    0xE8, 0xE8, 0xE8, 0xff, 0xff, 0xff, 0xE8, 0xE8, 0xE8, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xE8, 0xE8, 0xE8, 0xff, 0xff, 0xff, 0xE8, 0xE8, 0xE8,
};

/*
// red blue checker
GLubyte checker[] = {
    0x8B, 0x5F, 0x65, 0x4F, 0x94, 0xCD, 0x8B, 0x5F, 0x65, 0x4F, 0x94, 0xCD,
    0x4F, 0x94, 0xCD, 0x8B, 0x5F, 0x65, 0x4F, 0x94, 0xCD, 0x8B, 0x5F, 0x65,
    0x8B, 0x5F, 0x65, 0x4F, 0x94, 0xCD, 0x8B, 0x5F, 0x65, 0x4F, 0x94, 0xCD,
    0x4F, 0x94, 0xCD, 0x8B, 0x5F, 0x65, 0x4F, 0x94, 0xCD, 0x8B, 0x5F, 0x65,
};
*/

GLuint checker_IDs[3];
GLuint circles_IDs[3];

void
makeCheckerFloorTexture(bool useMipmaps, bool linearFiltering)
{

  if (useMipmaps) {
    glBindTexture(GL_TEXTURE_2D, checker_IDs[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
      GL_LINEAR_MIPMAP_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, 4, 4,
      GL_RGB, GL_UNSIGNED_BYTE, checker);
  } else {
    if (linearFiltering) {
      glBindTexture(GL_TEXTURE_2D, checker_IDs[1]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    } else {
      glBindTexture(GL_TEXTURE_2D, checker_IDs[2]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 4, 4, 0,
      GL_RGB, GL_UNSIGNED_BYTE, checker);
  }
}

void
makeCircleFloorTexture(bool useMipmaps, bool linearFiltering)
{
  GLubyte floorTexture[16][16][3];
  int s, t;

  /* Setup RGB image for the texture. */
  GLubyte *loc = (GLubyte*) floorTexture;
  for (t = 0; t < 16; t++) {
    for (s = 0; s < 16; s++) {
      if (circles[t][s] == 'x') {
        /* Nice green. */
        loc[0] = 0x1f;
        loc[1] = 0x8f;
        loc[2] = 0x1f;
      } else {
        /* Light gray. */
        loc[0] = 0xaa;
        loc[1] = 0xaa;
        loc[2] = 0xaa;
      }
      loc += 3;
    }
  }

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  if (useMipmaps) {
    glBindTexture(GL_TEXTURE_2D, circles_IDs[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
      GL_LINEAR_MIPMAP_LINEAR);
    gluBuild2DMipmaps(GL_TEXTURE_2D, 3, 16, 16,
      GL_RGB, GL_UNSIGNED_BYTE, floorTexture);
  } else {
    if (linearFiltering) {
      glBindTexture(GL_TEXTURE_2D, circles_IDs[1]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    } else {
      glBindTexture(GL_TEXTURE_2D, circles_IDs[2]);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    }
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 16, 16, 0,
      GL_RGB, GL_UNSIGNED_BYTE, floorTexture);
  }
}

/* Draw a floor (possibly textured). */
void
drawFloor(GLfloat floorVertices[4][3], 
	GLfloat tex_scale, bool useTexture)
// tex_scale = 4 if checker, =32 if circles
{
  glDisable(GL_LIGHTING);
 
  enum filters{filter0, filter1, filter2};

  if (useTexture) {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, checker_IDs[filter1]); //linear filtering
    //glBindTexture(GL_TEXTURE_2D, circles_IDs[filter1]); //linear filtering
  }

  glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3fv(floorVertices[0]);
    glTexCoord2f(0.0, tex_scale);
    glVertex3fv(floorVertices[1]);
    glTexCoord2f(tex_scale, tex_scale);
    glVertex3fv(floorVertices[2]);
    glTexCoord2f(tex_scale, 0.0);
    glVertex3fv(floorVertices[3]);
  glEnd();

  if (useTexture) {
    glDisable(GL_TEXTURE_2D);
  }

  glEnable(GL_LIGHTING);

}


}//namespace


