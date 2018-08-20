#include "info_methods/random_solver.h"

namespace info_planner {

  Random_Solver::Random_Solver() {

  }

  info_planner::info_path Random_Solver::solve(GP_Model gp_model, const size_t &stages, double &value, double &time) {
    info_path waypoints(stages);
    bool duplicate = true;
    while (duplicate) {
      for (size_t i = 0; i < stages; i++) {
        auto it = gp_model.point_set.begin();
        std::advance(it, rand() % gp_model.point_set.size());
        waypoints[i] = *it;
      }

      // checking for duplicates
      duplicate = false;
      for (size_t i = 0; i < stages; i++) {
        for (size_t j = i + 1; j < stages; j++) {
          if (waypoints[i].first == waypoints[j].first && waypoints[i].second == waypoints[j].second) {
            duplicate = true;
          }
        }
      }
    }

    // TODO: ignored
    value = 0;
    time = 0;

    return waypoints;
  }
}