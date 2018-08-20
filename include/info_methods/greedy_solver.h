#ifndef GREEDY_SOLVER_H
#define	GREEDY_SOLVER_H

#include "solver_interface.h"
namespace info_planner {

  class Greedy_Solver : public Solver_Interface {
  public:
    Greedy_Solver();
    info_path solve(GP_Model gp_model, const size_t &stages, double &value, double &time);

    const std::string &get_name() {
      return name;
    };

  private:
    const std::string name = "Greedy";

  };
}

#endif	/* GREEDY_SOLVER_H */

