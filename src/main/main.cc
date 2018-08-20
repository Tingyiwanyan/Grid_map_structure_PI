
#include "animation/animation.h"
#include <pthread.h>


int
main(int argc, char** argv) {

  int i=pthread_getconcurrency();
  /*
   Above line is useless but just fix the seg-fault caused by nvidia driver, see 
   https://bugs.launchpad.net/ubuntu/+source/nvidia-graphics-drivers-319/+bug/1248642 
  */

  animation(argc, argv);

  return 0;

}

