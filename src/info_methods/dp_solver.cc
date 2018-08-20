#include "info_methods/dp_solver.h"

namespace info_planner {

  DP_Solver::DP_Solver() {

  }

  info_planner::info_path DP_Solver::solve(GP_Model gp_model, const size_t &stages, double &value, double &time) {
    std::vector<std::vector<std::vector<double>>> V(stages); // n * nc_data.n_rows * nc_data.n_cols
    for (auto& v2 : V) {
      v2.resize(gp_model.nc_data.n_rows);
      for (auto& v : v2) {
        v.resize(gp_model.nc_data.n_cols, std::numeric_limits<double>::lowest());
      }
    }

    std::vector<std::vector<std::vector<gp_point>>> back_track(stages); // n * nc_data.n_rows * nc_data.n_cols
    for (auto& v2 : back_track) {
      v2.resize(gp_model.nc_data.n_rows);
      for (auto& v : v2) {
        v.resize(gp_model.nc_data.n_cols);
      }
    }
    std::cout << "gp_model.point_set.size() " << gp_model.point_set.size() << std::endl;
    for (size_t i = 0; i < stages; i++) {
      std::cout << "DP i " << i << std::endl;
      if (i == 0) {
        for (const auto& point : gp_model.point_set) {
          Eigen::VectorXi X(2);
          X << point.first, point.second;
          std::set<gp_point> temp_u_set = gp_model.point_set;
          temp_u_set.erase(point);

          Eigen::MatrixXi curr_U(temp_u_set.size(), 2);
          int j = 0;
          for (const auto& p : temp_u_set) {
            curr_U(j, 0) = p.first;
            curr_U(j, 1) = p.second;
            j++;
          }

          const clock_t begin_time = clock();
          V[i][point.first][point.second] = gp_model.MutualInformation(X, curr_U);
          time += static_cast<double> (clock() - begin_time) / CLOCKS_PER_SEC;
        }

      } else {
        std::vector<std::pair<gp_point, std::pair<double, double>>> temp_samples = gp_model.get_samples();
        for (const auto& prev_point : gp_model.point_set) {
          std::set<gp_point> temp_u_set = gp_model.point_set;

          // Remove previous path from point_set
          gp_point temp_prev_point = prev_point;
          for (int j = i - 1; j >= 0; j--) {
            size_t map_x[] = {temp_prev_point.first, temp_prev_point.second};
            gp_model.add_pattern(map_x, 0);

            temp_u_set.erase(temp_prev_point);
            temp_prev_point = back_track[j][temp_prev_point.first][temp_prev_point.second];
          }

          for (const auto& point : temp_u_set) {
            std::set<gp_point> temp_temp_u_set = temp_u_set;

            Eigen::VectorXi X(2);
            X << point.first, point.second;
            temp_temp_u_set.erase(point);

            Eigen::MatrixXi rest_U(temp_temp_u_set.size(), 2);
            size_t j = 0;
            for (const auto& p : temp_temp_u_set) {
              rest_U(j, 0) = p.first;
              rest_U(j, 1) = p.second;
              j++;
            }

            const clock_t begin_time = clock();
            double new_value = gp_model.MutualInformation(X, rest_U) + V[i - 1][prev_point.first][prev_point.second];
            time += static_cast<double> (clock() - begin_time) / CLOCKS_PER_SEC;

            if (new_value > V[i][point.first][point.second]) {
              V[i][point.first][point.second] = new_value;
              back_track[i][point.first][point.second] = prev_point;
            }
          }

          // Reset the model
          gp_model.clear_samples();
          for (const auto& s : temp_samples) {
            size_t x[] = {s.first.first, s.first.second};
            gp_model.add_pattern(x, s.second.first);
          }
        }
      }
    }

    std::cout << "DP time " << time << std::endl;

    double max = std::numeric_limits<double>::lowest();
    gp_point coordinate;
    for (const auto& point : gp_model.point_set) {
      if (V[stages - 1][point.first][point.second] > max) {
        max = V[stages - 1][point.first][point.second];
        coordinate = point;
      }
    }

    value = max;
    std::cout << std::fixed << "DP total_I " << value << std::endl;

    info_path path;
    path.push_back(coordinate);
    for (int i = stages - 2; i >= 0; i--) {
      path.insert(path.begin(), back_track[i + 1][coordinate.first][coordinate.second]);
      coordinate = path[0];
    }

    return path;
  }
}