
#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <yaml-cpp/yaml.h>


namespace utils
{


class Parameters
{

public:

  typedef boost::shared_ptr<Parameters> Ptr;
  typedef boost::shared_ptr<const Parameters> ConstPtr;

  Parameters(std::string&);
  virtual ~Parameters();

  YAML::Node getParamNode(void){ return params; }


protected:

  YAML::Node params;

};


}//namespace

#endif

