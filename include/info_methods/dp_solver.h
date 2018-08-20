#ifndef DP_SOLVER_H
#define	DP_SOLVER_H

#include "solver_interface.h"

namespace info_planner {

  class DP_Solver : public Solver_Interface {
  public:
    DP_Solver();
    info_path solve(GP_Model gp_model, const size_t &stages, double &value, double &time);

    const std::string &get_name() {
      return name;
    };

  private:
    const std::string name = "DP";
  };
}

#endif	/* DP_SOLVER_H */

