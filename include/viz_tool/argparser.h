

#ifndef ARGPARSER_H
#define ARGPARSER_H

#include <stdio.h>
#include <assert.h>
#include "viz_tool/glm.h"

//class ArgParser{
//public:

extern char*    Model_File;
extern size_t   RandSeed;
extern size_t   Num_agents;

void argParser(int argc, char** argv);

//};


#endif

