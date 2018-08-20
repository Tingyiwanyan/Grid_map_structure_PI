#ifndef SOLVER_INTERFACE_H
#define	SOLVER_INTERFACE_H

#include <vector>
#include <utility>
#include "gp_model.h"

namespace info_planner {
  typedef std::vector<gp_point> info_path;
  
  class Solver_Interface {
  public:
    virtual info_path solve(GP_Model gp_model, const size_t &stages, double &value, double &time) = 0;
    virtual const std::string &get_name() = 0;
  };
}

#endif

