#include <iostream>
#include "utils/parameters.h"

namespace utils
{


Parameters::Parameters(std::string& input_file){

  params = YAML::LoadFile(input_file.c_str());
  std::cout<<"yaml file "<<input_file<<" successfully loaded."<<std::endl;

}

Parameters::~Parameters(void){}


}//namespace



