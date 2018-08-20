#include "info_methods/greedy_solver.h"

namespace info_planner {

  Greedy_Solver::Greedy_Solver() {

  }

  info_path Greedy_Solver::solve(GP_Model gp_model, const size_t &stages, double &value, double &time) {
    info_path path;
    std::set<gp_point> temp_point_set = gp_model.point_set;

    for (int i = 0; i < stages; i++) {
      std::cout << "Greedy i " << i << std::endl;

      double max = std::numeric_limits<double>::lowest();

      gp_point max_point;
      for (const auto& point : temp_point_set) {
        Eigen::VectorXi X(2);
        X << point.first, point.second;

        std::set<gp_point> u_set = temp_point_set;
        u_set.erase(point);

        Eigen::MatrixXi curr_U(u_set.size(), 2);
        int j = 0;
        for (const auto& p : u_set) {
          curr_U(j, 0) = p.first;
          curr_U(j, 1) = p.second;
          j++;
        }

        const clock_t begin_time = clock();
        //      std::cout << "X " << X << std::endl;
        //      std::cout << "curr_U " << curr_U << std::endl;
        double value = gp_model.MutualInformation(X, curr_U);
        time += static_cast<double> (clock() - begin_time) / CLOCKS_PER_SEC;
//        std::cout << "static_cast<double> (clock() - begin_time) / CLOCKS_PER_SEC " << static_cast<double> (clock() - begin_time) / CLOCKS_PER_SEC << std::endl;

        if (value > max) {
          max = value;
          max_point = point;
        }
      }

      value += max;
      temp_point_set.erase(max_point);
      path.push_back(max_point);
      size_t map_x[] = {max_point.first, max_point.second};
      gp_model.add_pattern(map_x, 0);
    }


    std::cout << "Greedy time " << time << std::endl;
    std::cout << std::fixed << "Greedy total_I " << value << std::endl;
    return path;
  }
}
