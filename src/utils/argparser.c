
#include "utils/argparser.h"
#include <string.h>

char*      Model_File = (char*)"data/mesh_objs/create_model.obj";
size_t     RandSeed = 1;
size_t     Num_agents = 6;

void argParser(int argc, char** argv){

    for (int i = 1; i < argc; i++) {
      if (!strcmp(argv[i], "-i") || !strcmp(argv[i], "-input")) {
        i++; assert (i < argc);
        Model_File = argv[i];
      } else if (!strcmp(argv[i], "-s") || !strcmp(argv[i], "-seed")) {
        i++; assert (i < argc);
        RandSeed = atoi(argv[i]);
      } else if (!strcmp(argv[i], "-n") || !strcmp(argv[i], "-size")) {
        i++; assert (i < argc);
        Num_agents = atoi(argv[i]);
      } else {
        printf ("whoops error with command line argument %d: '%s'\n",i,argv[i]);
        assert(0);
      }
    }

}


