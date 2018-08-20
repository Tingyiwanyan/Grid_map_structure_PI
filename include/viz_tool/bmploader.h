
/******************************************************************************
* File: BMPloader.h
* Description: loading the bmp format image, so as to use it as texture. Part of the code is from Nehe contributors
* Author: Lantao Liu
* Date: 6/2010 
******************************************************************************/


#ifndef BMPLOADER_H
#define BMPLOADER_H

#include <stdio.h>      // Header file for standard file i/o.
#include <stdlib.h>     // Header file for malloc/free.

namespace viz_tool
{

  /* Image type - contains height, width, and data */
  struct Image {
      //unsigned long sizeX;
      //unsigned long sizeY;
      //change: 2015/09/28, bug report: http://cboard.cprogramming.com/c-programming/95644-malloc-issues.html
      int sizeX;	
      int sizeY;
      char *data;
  };

  typedef struct Image Image;

  int ImageLoad(char *filename, Image *image);

}

#endif

