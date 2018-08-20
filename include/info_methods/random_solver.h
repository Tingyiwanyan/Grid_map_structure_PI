#ifndef RANDOM_SOLVER_H
#define RANDOM_SOLVER_H

#include "solver_interface.h"

namespace info_planner {

  class Random_Solver : public Solver_Interface {
  public:
    Random_Solver();
    info_path solve(GP_Model gp_model, const size_t &stages, double &value, double &time);

    const std::string &get_name() {
      return name;
    };

  private:
    const std::string name = "Random";

  };
}

#endif /* RANDOM_SOLVER_H */

