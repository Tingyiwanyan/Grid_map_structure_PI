#include <iomanip>
#include "info_methods/info_core.h"

#include "info_methods/tsp_brute.h"

#include <dlib/optimization.h>
#include <dlib/matrix.h>

#include "hungarian/Hungarian.h"
#include "hungarian/BipartiteGraph.h"

std::string dir_name;

namespace info_planner {

  Info::Info(const utils::Parameters::Ptr& pParams, const std::vector<double> &cov_params, const size_t &layers, const size_t &grid_x_size, const size_t &grid_y_size, const utils::nc_data_t &nc_data, const info_path &starting_points, const std::set<gp_point> &observations, Solver_Interface *solver)
  : nc_data(nc_data), starting_points(starting_points), observations(observations), layers(layers), grid_x_size(grid_x_size), grid_y_size(grid_y_size), solver(solver), image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(255, 255, 255)), q_goals(starting_points.size()) {
    assert(pParams);
    this->pParams = pParams;

    loadParams();
    this->cov_params = cov_params;
    gp_model_var = new GP_Model(nc_data, solver->get_name(), input_dim, cov_func, cov_params, max_bv_size, tolerance_error, observations, true);
  }

  Info::~Info() {
    if (gp_model_var) {
      delete gp_model_var;
    }
  }

  void Info::loadParams(void) {
    input_dim = pParams->getParamNode()["info_plan"]["gp_params"]["input_dim"].as<size_t>();
    cov_func = pParams->getParamNode()["info_plan"]["gp_params"]["cov_func"].as<std::string>();
    max_bv_size = pParams->getParamNode()["info_plan"]["gp_params"]["max_bv_size"].as<size_t>();
    tolerance_error = pParams->getParamNode()["info_plan"]["gp_params"]["tolerance_error"].as<double>();

    lambda = pParams->getParamNode()["info_plan"]["multi_robot"]["lambda"].as<double>();
    rounds = pParams->getParamNode()["info_plan"]["multi_robot"]["rounds"].as<size_t>();

  }

  info_paths Info::multiPath(info_path final_way_points) {
    for (int j = 0; j < final_way_points.size(); j++) {
      std::cout << "final_way_points[j] " << final_way_points[j].first << " " << final_way_points[j].second << std::endl;
    }

    const int max_length = std::sqrt(std::pow(nc_data.n_rows, 2) + std::pow(nc_data.n_cols, 2)) + 1;

    info_path ending_points(num_robots);
    ending_points[0] = final_way_points[3];
    ending_points[1] = final_way_points[4];
    //  ending_points[0] = std::make_pair(nc_data.n_rows - 1, 0);
    //  ending_points[1] = std::make_pair(nc_data.n_rows - 1, 0);

    for (int i = 0; i < num_robots; i++) {
      final_way_points.erase(std::find(final_way_points.begin(), final_way_points.end(), ending_points[i]));
    }

    for (int i = 0; i < num_robots; i++) {
      std::cout << "starting_point[i] " << starting_points[i].first << " " << starting_points[i].second << std::endl;
      for (int j = 0; j < final_way_points.size(); j++) {
        std::cout << "final_way_points[j] " << final_way_points[j].first << " " << final_way_points[j].second << std::endl;
      }
      std::cout << "ending_points[i] " << ending_points[i].first << " " << ending_points[i].second << std::endl;
    }

    // Utility matrix
    dlib::matrix<int> d_m;
    d_m.set_size(final_way_points.size() + num_robots, final_way_points.size() + num_robots);
    for (int i = 0; i < final_way_points.size(); i++) {
      for (int j = 0; j < final_way_points.size(); j++) {
        d_m(i, j) = -std::sqrt(std::pow(final_way_points[i].first - final_way_points[j].first, 2) + std::pow(final_way_points[i].second - final_way_points[j].second, 2));
      }
    }

    for (int i = 0; i < num_robots; i++) {
      for (int j = 0; j < final_way_points.size(); j++) {
        d_m(final_way_points.size() + i, j) = -std::sqrt(std::pow(starting_points[i].first - final_way_points[j].first, 2) + std::pow(starting_points[i].second - final_way_points[j].second, 2));
        d_m(j, final_way_points.size() + i) = -std::sqrt(std::pow(ending_points[i].first - final_way_points[j].first, 2) + std::pow(ending_points[i].second - final_way_points[j].second, 2));
      }

      for (int j = 0; j < num_robots; j++) {
        d_m(final_way_points.size() + i, final_way_points.size() + j) = -std::sqrt(std::pow(starting_points[i].first - ending_points[j].first, 2) + std::pow(starting_points[i].second - ending_points[j].second, 2));
      }

      //    d_m(final_way_points.size() + i, final_way_points.size() + i) = -INF; //???
    }

    // k-nearest
    for (int i = 0; i < final_way_points.size() + num_robots; i++) {
      int k_neareast = 3;
      if (i >= final_way_points.size()) {
        k_neareast = 3;
      }

      std::priority_queue<std::pair<int, int>> q;
      for (int j = 0; j < final_way_points.size() + num_robots; j++) {
        q.push(std::pair<int, int>(d_m(i, j), j));
      }
      for (int j = 0; j < k_neareast; j++) {
        q.pop();
      }
      while (!q.empty()) {
        d_m(i, q.top().second) = -max_length;
        q.pop();
      }
    }
    // directed to undirected
    for (int i = 0; i < final_way_points.size(); i++) {
      for (int j = 0; j < final_way_points.size(); j++) {
        if (d_m(i, j) != -max_length) {
          d_m(j, i) = d_m(i, j);
        }
      }
    }

    // draw graph
    cv::Mat graph_image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < final_way_points.size(); i++) {
      cv::Point point1(cv::Point(final_way_points[i].second, graph_image.rows - final_way_points[i].first - 1));

      for (int j = 0; j < final_way_points.size(); j++) {
        if (d_m(i, j) != -max_length) {
          cv::Point point2(cv::Point(final_way_points[j].second, graph_image.rows - final_way_points[j].first - 1));
          cv::line(graph_image, point1, point2, cv::Scalar(0, 255, 0), 4);
        }
      }

      for (int j = 0; j < num_robots; j++) {
        if (d_m(i, final_way_points.size() + j) != -max_length) {
          cv::Point point2(cv::Point(ending_points[j].second, graph_image.rows - ending_points[j].first - 1));
          cv::line(graph_image, point1, point2, cv::Scalar(0, 255, 0), 4);
          cv::circle(graph_image, point2, 12, cv::Scalar(0, 255, 255), -1);
        }
      }
      cv::circle(graph_image, point1, 12, cv::Scalar(255, 0, 0), -1);
    }

    for (int i = 0; i < num_robots; i++) {
      cv::Point point1(cv::Point(starting_points[i].second, graph_image.rows - starting_points[i].first - 1));
      for (int j = 0; j < final_way_points.size(); j++) {
        if (d_m(final_way_points.size() + i, j) != -max_length) {
          cv::Point point2(cv::Point(final_way_points[j].second, graph_image.rows - final_way_points[j].first - 1));
          cv::line(graph_image, point1, point2, cv::Scalar(0, 255, 0), 4);
        }
      }

      for (int j = 0; j < num_robots; j++) {
        if (d_m(final_way_points.size() + i, final_way_points.size() + j) != -max_length) {
          cv::Point point2(cv::Point(ending_points[j].second, graph_image.rows - ending_points[j].first - 1));
          cv::line(graph_image, point1, point2, cv::Scalar(0, 255, 0), 4);
          cv::circle(graph_image, point2, 12, cv::Scalar(0, 255, 255), -1);
        }
      }
      cv::circle(graph_image, point1, 12, cv::Scalar(0, 255, 0), -1);
    }
    cv::imwrite(dir_name + "/" + std::to_string(observations.size()) + "observations_" + solver->get_name() + "_graph.png", graph_image);
    // draw graph

    std::cout << "d_m\n" << d_m << std::endl;
    vector<long> asgn_vec = dlib::max_cost_assignment(d_m);
    for (int j = 0; j < asgn_vec.size(); j++) {
      std::cout << "asgn_vec[j] " << asgn_vec[j] << std::endl;
    }

    // transform matching to a path.
    info_paths final_paths(num_robots);
    for (int i = 0; i < num_robots; i++) {
      info_path matching_path;

      long mathching_index = asgn_vec[final_way_points.size() + i];
      while (true) {
        std::cout << "mathching_index " << mathching_index << std::endl;
        if (mathching_index >= final_way_points.size()) {
          matching_path.push_back(ending_points[mathching_index - final_way_points.size()]); // different starts
          break;
        } else {
          matching_path.push_back(final_way_points[mathching_index]);
        }
        mathching_index = asgn_vec[mathching_index];
      }

      //    matching_path.insert(matching_path.begin(), starting_points[i]); // don't include the starting_point
      for (int j = 0; j < matching_path.size(); j++) {
        std::cout << "matching_path[j] " << matching_path[j].first << " " << matching_path[j].second << std::endl;
      }
      final_paths[i] = matching_path;
    }

    return final_paths;
  }

  info_paths Info::incrementalMultiPath(const info_path &waypoints, double &time) {
    for (int i = 0; i < waypoints.size(); i++) {
      std::cout << "way_points[i] " << waypoints[i].first << " " << waypoints[i].second << std::endl;
    }

    //  const unsigned int num_robots = starting_points.size();
    const int max_length = std::sqrt(std::pow(nc_data.n_rows, 2) + std::pow(nc_data.n_cols, 2)) + 1;
    std::cout << "starting_points.size() " << starting_points.size() << std::endl;
    for (unsigned int i = 0; i < num_robots; i++) {
      std::cout << "starting_points[i].first " << starting_points[i].first << std::endl;
      std::cout << "starting_points[i].second " << starting_points[i].second << std::endl;
    }

    // draw graph
    cv::Mat graph_image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < waypoints.size(); i++) {
      cv::Point point1(cv::Point(waypoints[i].second, graph_image.rows - waypoints[i].first - 1));

      for (int j = 0; j < waypoints.size(); j++) {
        //      if (d_m(i, j) != -max_length) {
        cv::Point point2(cv::Point(waypoints[j].second, graph_image.rows - waypoints[j].first - 1));
        cv::line(graph_image, point1, point2, cv::Scalar(0, 255, 0), 4);
        //      }
      }

      for (int j = 0; j < num_robots; j++) {
        //      if (d_m(i, final_way_points.size() + j) != -max_length) {
        //        cv::Point point2(cv::Point(ending_points[j].second, graph_image.rows - ending_points[j].first - 1));
        //        cv::line(graph_image, point1, point2, cv::Scalar(255, 0, 255));
        //        cv::circle(graph_image, point2, 10, cv::Scalar(0, 255, 255), -1);
        //      }
      }
      cv::circle(graph_image, point1, 12, cv::Scalar(255, 0, 0), -1);
    }
    for (int i = 0; i < num_robots; i++) {
      cv::Point point1(cv::Point(starting_points[i].second, graph_image.rows - starting_points[i].first - 1));
      for (int j = 0; j < waypoints.size(); j++) {
        //      if (d_m(waypoints.size() + i, j) != -max_length) {
        cv::Point point2(cv::Point(waypoints[j].second, graph_image.rows - waypoints[j].first - 1));
        cv::line(graph_image, point1, point2, cv::Scalar(0, 255, 0), 4);
        //      }
      }

      for (int j = 0; j < num_robots; j++) {
        //      if (d_m(waypoints.size() + i, waypoints.size() + j) != -max_length) {
        //        cv::Point point2(cv::Point(ending_points[j].second, graph_image.rows - ending_points[j].first - 1));
        //        cv::line(graph_image, point1, point2, cv::Scalar(255, 0, 255));
        //        cv::circle(graph_image, point2, 10, cv::Scalar(0, 255, 255), -1);
        //      }
      }
      cv::circle(graph_image, point1, 12, cv::Scalar(0, 255, 0), -1);
    }
    cv::imwrite(dir_name + "/" + std::to_string(observations.size()) + "observations_" + solver->get_name() + "_graph.png", graph_image);

    // Initialization
    std::vector<unsigned int> prev_matchings_indices;
    std::vector<unsigned int> remaining_indices(waypoints.size());
    for (unsigned int i = 0; i < waypoints.size(); i++) {
      remaining_indices[i] = i;
    }
    std::vector<hungarian::EID> final_matchings;

    // Utility matrix
    for (unsigned int i = 0; i < rounds; i++) {
      hungarian::Matrix utility_matrix(prev_matchings_indices.size() + num_robots); // rows
      for (unsigned int j = 0; j < utility_matrix.size(); j++) {
        utility_matrix[j].resize(waypoints.size()); // columns

        double x1 = 0;
        double y1 = 0;
        if (j < prev_matchings_indices.size()) { // previous matching points
          x1 = waypoints[prev_matchings_indices[j]].first;
          y1 = waypoints[prev_matchings_indices[j]].second;
        } else { // remaining waypoints
          x1 = starting_points[j - prev_matchings_indices.size()].first;
          y1 = starting_points[j - prev_matchings_indices.size()].second;
        }

        for (unsigned int k = 0; k < prev_matchings_indices.size(); k++) {
          double x2 = waypoints[prev_matchings_indices[k]].first;
          double y2 = waypoints[prev_matchings_indices[k]].second;
          double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
          utility_matrix[j][k].SetWeight(-distance);
        }

        for (unsigned int k = 0; k < remaining_indices.size(); k++) {
          double x2 = waypoints[remaining_indices[k]].first;
          double y2 = waypoints[remaining_indices[k]].second;
          double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
          utility_matrix[j][prev_matchings_indices.size() + k].SetWeight(-distance);
        }
      }

      // Parameterize on the diagonal of previous matchings
      for (unsigned int j = 0; j < prev_matchings_indices.size(); j++) {
        double min = max_length;
        utility_matrix[j][j].SetWeight(max_length);
        for (unsigned int k = 0; k < utility_matrix[j].size(); k++) {
          min = std::min(utility_matrix[j][k].GetWeight(), min);
        }
        utility_matrix[j][j].SetWeight(min * lambda);
      }

      //define a bipartite graph
      hungarian::BipartiteGraph bg(utility_matrix);

      //run Hungarian methodu
      hungarian::Hungarian h(bg);
      std::vector<hungarian::EID> matchings(utility_matrix.size());
      const clock_t begin_time = clock();
      h.HungarianAlgo(matchings);
      time += static_cast<double> (clock() - begin_time) / CLOCKS_PER_SEC;
      //    h.DisplayConfig(bg);

      if (i == rounds - 1) {
        final_matchings = matchings;
        break;
      }

      std::vector<unsigned int> temp_matchings_indices;
      temp_matchings_indices.reserve(utility_matrix.size());
      for (const auto &p : matchings) {
        std::cout << "p " << p.first << " " << p.second << std::endl;
        if (p.second < prev_matchings_indices.size()) {
          temp_matchings_indices.push_back(prev_matchings_indices[p.second]);
        } else {
          temp_matchings_indices.push_back(remaining_indices[p.second - prev_matchings_indices.size()]);
        }
      }
      prev_matchings_indices = temp_matchings_indices;
      remaining_indices.clear();
      remaining_indices.reserve(waypoints.size() - prev_matchings_indices.size());
      for (unsigned int j = 0; j < waypoints.size(); j++) {
        bool remain = true;
        for (unsigned int k = 0; k < prev_matchings_indices.size(); k++) {
          if (prev_matchings_indices[k] == j) {
            remain = false;
            break;
          }
        }
        if (remain) {
          remaining_indices.push_back(j);
        }
      }

      //    std::cout << "prev_matchings_indices.size() " << prev_matchings_indices.size() << std::endl;
      //    for (const auto &index : prev_matchings_indices) {
      //      std::cout << "index " << index << " point  " << waypoints[index].first << " " << waypoints[index].second << std::endl;
      //    }
      //
      //    std::cout << "remaining_indices.size() " << remaining_indices.size() << std::endl;
      //    for (const auto &index : remaining_indices) {
      //      std::cout << "index " << index << " point  " << waypoints[index].first << " " << waypoints[index].second << std::endl;
      //    }
    }

    // transform matching to a path.
    //  std::cout << "final_matchings.size() " << final_matchings.size() << std::endl;
    info_paths final_paths(num_robots);
    for (size_t i = 0; i < num_robots; i++) {
      info_path matching_path;

      unsigned int mathching_index = final_matchings[final_matchings.size() - num_robots + i].second;
      while (true) {
        //      std::cout << "mathching_index " << mathching_index << std::endl;
        if (mathching_index >= final_matchings.size() - num_robots) {
          matching_path.push_back(waypoints[remaining_indices[mathching_index - prev_matchings_indices.size()]]);
        } else {
          matching_path.push_back(waypoints[prev_matchings_indices[mathching_index]]);
        }

        if (mathching_index >= final_matchings.size() - num_robots) {
          break;
        }
        mathching_index = final_matchings[mathching_index].second;
      }

      //    matching_path.insert(matching_path.begin(), starting_points[i]); // don't include the starting_point
      //    std::cout << "matching_path.size() " << matching_path.size() << std::endl;
      //    for (int j = 0; j < matching_path.size(); j++) {
      //      std::cout << "matching_path[j] " << matching_path[j].first << " " << matching_path[j].second << std::endl;
      //    }
      //    std::cout << "------------------------" << std::endl;
      final_paths[i] = matching_path;
    }
    return final_paths;
  }

  info_paths Info::singlePath(info_path final_way_points) {
//    final_way_points.clear();
    final_way_points.insert(final_way_points.begin(), starting_points[0]);
//    final_way_points.push_back(gp_point(100, 200));
//    final_way_points.push_back(gp_point(70, 200));
//    final_way_points.push_back(gp_point(40, 200));
//    final_way_points.push_back(gp_point(10, 200));
//    final_way_points.push_back(gp_point(130, 150));
    info_path shortest_path = final_way_points;
    if (solver->get_name() != "Greedy") {
      shortest_path = tsp_tour(final_way_points);
    }

    //    drawPath(shortest_path);
    //    cv::imwrite(dir_name + std::to_string(observations.size()) + "observations_" + std::to_string(stages) + solver->get_name() + ".png", image);
    //
    //    WriteLengthResult(shortest_path, stages);

    size_t index = 0;
    for (size_t i = 0; i < shortest_path.size(); i++) {
//      std::cout << shortest_path[i].first << " " << shortest_path[i].second << std::endl;
      if (shortest_path[i].first == starting_points[0].first && shortest_path[i].second == starting_points[0].second) {
        index = i;
        break;
      }
    }
    info_paths final_shortest_path(1);
    for (size_t i = 0; i < shortest_path.size(); i++) {
      if (index == shortest_path.size()) {
        index = 0;
      }
      final_shortest_path[0].push_back(shortest_path[index]);
      index++;
    }

    return final_shortest_path;
  }
  
  void Info::pred(const info_paths &paths) {
    std::set<gp_point> observations;
    for (const info_path &path : paths) {
      for (const gp_point &point : path) {
        observations.insert(point);
      }
    }
    GP_Model gp_model(nc_data, solver->get_name(), input_dim, cov_func, cov_params, max_bv_size, tolerance_error, observations, true);

    // total squared error
    double tss = 0;
    for (size_t i = 0; i < nc_data.n_rows; i++) {
      for (size_t j = 0; j < nc_data.n_cols; j++) {
        double y = nc_data.frames[0].cells[i][j].salinity;
        if (std::isnan(y)) {
          continue;
        }
        size_t x[] = {i, j};
        double f = gp_model.f(x);
        // std::cout << "f " << f << std::endl;
        // std::cout << "y " << y << std::endl;
        double error = f - y;
        tss += error*error;
      }
    }
    std::cout << "mse = " << tss / (nc_data.n_rows * nc_data.n_cols) << std::endl;
  } 
  
  info_paths Info::run(const utils::nc_data_t &nc_data, const size_t &frame_count, const size_t &stages, const size_t &z) {
    std::cout << "test " << z << std::endl;

    info_path final_way_points;
    double total_time = 0;
    double total_mi_value = 0;
    final_way_points = recursive(nc_data, stages, 0, 0, total_mi_value, total_time);
    WriteTotalMutualInformation(total_mi_value, stages);
    WriteTimeResult(total_time, stages, z);

    info_paths final_paths;
    for (double j = 1.0; j <= 1.0; j += 0.2) {
      lambda = j;
      for (size_t k = stages / num_robots; k <= stages / num_robots; k++) {
        image = cv::Mat(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(0, 0, 0));
        drawBackGround(frame_count);
        rounds = k;
        const bool multi_robot = false;
        if (multi_robot) {
          //    final_paths = multiPath(final_way_points);
          double total_time_path = 0;
          final_paths = incrementalMultiPath(final_way_points, total_time_path);
          WritePathTimeResult(total_time_path, stages, z);
        } else {
          final_paths = singlePath(final_way_points);
          for (size_t i = 0; i < final_paths[0].size(); i++) {
            std::cout << "ggg " << final_paths[0][i].first << " " << final_paths[0][i].second << std::endl;
          }
        }
        
        final_paths_draw = final_paths;

        for (size_t i = 0; i < final_paths.size(); i++) {
          // If final_path does not include  starting_point
          info_path temp = final_paths[i];
          temp.push_back(starting_points[i]);
//          temp.insert(temp.begin(), );
          
          // TODO: workaround for drawing
          int start_x = static_cast<int> (temp[0].first);
          int start_y = static_cast<int> (temp[0].second);

          int x1 = static_cast<int> (temp[1].first);
          int y1 = static_cast<int> (temp[1].second);

          int x2 = static_cast<int> (temp[temp.size() - 2].first);
          int y2 = static_cast<int> (temp[temp.size() - 2].second);

          info_planner::info_path final_path;
          if (std::sqrt(std::pow(start_x - x1, 2) + std::pow(start_y - y1, 2)) < std::sqrt(std::pow(start_x - x2, 2) + std::pow(start_y - y2, 2))) {
            for (size_t i = 0; i < temp.size() - 1; i++) {
//              std::cout << "gggg " << temp[i].first << " " << temp[i].second << std::endl;
              final_path.push_back(temp[i]);
            }
          } else {
            for (size_t i = temp.size() - 1; i >= 1; i--) {
//              std::cout << "gggg " << temp[i].first << " " << temp[i].second << std::endl;
              final_path.push_back(temp[i]);
            }
          }
          //

          drawPath(final_path);
        }
        cv::imwrite(dir_name + "/" + std::to_string(num_robots) + "robots_" + std::to_string(observations.size()) + "observations_" + std::to_string(stages) + solver->get_name() + "_round" + std::to_string(rounds) + "_lambda" + std::to_string(lambda) + "_" + std::to_string(z) + ".png", image);
      }
    }
    return final_paths;
  }
  
  void Info::draw(const gp_point &current_point, const std::vector<std::pair<gp_point, std::pair<double, double>>> &BV, const size_t &total_samples, const size_t &frame_count, const size_t &skip) {
    cv::Mat video_image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < nc_data.n_rows; i++) {
      for (int j = 0; j < nc_data.n_cols; j++) {
        double y = nc_data.frames[frame_count].cells[i][j].salinity;
        if (std::isnan(y)) {
          continue;
        }
        double value = 255.0 - (y - nc_data.min) / nc_data.range * 255.0;
        if (value < 0) {
          value = 0;
        } else if (value > 255.0) {
          value = 255.0;
        }
        video_image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(value, value, value);
      }
    }

    for (const auto &o : BV) {
      cv::circle(video_image, cv::Point(o.first.second, nc_data.n_rows - o.first.first - 1), 3, cv::Scalar(0, 255, 255), -1);
      //      image.at<cv::Vec3b>(nc_data.n_rows - o.first - 1, o.second) = cv::Vec3b(0.0, 255.0, 255.0);
    }


    for (size_t i = 0; i < final_paths_draw.size(); i++) {
      // If final_path does not include  starting_point
      info_path temp = final_paths_draw[i];
      temp.push_back(current_point);
      
      // TODO: workaround for drawing
      int start_x = static_cast<int> (temp[0].first);
      int start_y = static_cast<int> (temp[0].second);

      int x1 = static_cast<int> (temp[1].first);
      int y1 = static_cast<int> (temp[1].second);

      int x2 = static_cast<int> (temp[temp.size() - 2].first);
      int y2 = static_cast<int> (temp[temp.size() - 2].second);

      info_planner::info_path final_path;
      if (std::sqrt(std::pow(start_x - x1, 2) + std::pow(start_y - y1, 2)) < std::sqrt(std::pow(start_x - x2, 2) + std::pow(start_y - y2, 2))) {
        for (size_t i = 0; i < temp.size() - 1; i++) {
//          std::cout << "gggg " << temp[i].first << " " << temp[i].second << std::endl;
          final_path.push_back(temp[i]);
        }
      } else {
        for (size_t i = temp.size() - 1; i >= 1; i--) {
//          std::cout << "gggg " << temp[i].first << " " << temp[i].second << std::endl;
          final_path.push_back(temp[i]);
        }
      }
      //
      
      for (size_t i = 1; i <= skip; i++) {
        final_path.erase(final_path.begin() + 1);
      }

      for (size_t i = 0; i < final_path.size(); i++) {
        if (i == 0) {
          cv::circle(video_image, cv::Point(final_path[i].second, nc_data.n_rows - final_path[i].first - 1), 12, cv::Scalar(0, 0, 255), -1);
        } else {
          cv::circle(video_image, cv::Point(final_path[i].second, nc_data.n_rows - final_path[i].first - 1), 12, cv::Scalar(255, 0, 0), -1);
        }
        if (i < final_path.size() - 1) {
          cv::Point point1(final_path[i].second, nc_data.n_rows - final_path[i].first - 1);
          cv::Point point2(final_path[i + 1].second, nc_data.n_rows - final_path[i + 1].first - 1);
          cv::line(video_image, point1, point2, cv::Scalar(0, 255, 0), 4);
        }
      }
      
    }
    std::string output_name = "video_";
    for (size_t i = 0; i < 3 - std::log10(total_samples); i++) {
      output_name += "0";
    }
    output_name += std::to_string(total_samples);
    cv::imwrite(dir_name + "/" + output_name + ".png", video_image);

  }

  info_path Info::recursive(const utils::nc_data_t &original_nc_data, const size_t &stages, size_t current_layer, const size_t &point_num, double &total_mi_value, double &total_time) {
    double scaling_x = grid_x_size / static_cast<double> (original_nc_data.n_cols);
    double scaling_y = grid_y_size / static_cast<double> (original_nc_data.n_rows);

    utils::nc_data_t nc_data = original_nc_data;
    if (scaling_x != 1.0 || scaling_y != 1.0) {
      nc_data = utils::resamplingNC(original_nc_data, scaling_x, scaling_y);
    }

    std::set<gp_point> scaled_observations;
    for (const auto &o : observations) {
      scaled_observations.insert(gp_point(o.first * scaling_y, o.second * scaling_x));
      //    o.first *= scaling_y;
      //    o.second *= scaling_x;
    }

    GP_Model gp_model(nc_data, solver->get_name(), input_dim, cov_func, cov_params, max_bv_size, tolerance_error, scaled_observations, current_layer == 0);

    double mi_value = 0;
    double time = 0;
    info_planner::info_path path = solver->solve(gp_model, stages, mi_value, time);
    total_mi_value += mi_value;
    total_time += time;

    info_planner::info_path way_points;
    if (current_layer + 1 < layers) {
      double mi_value = 0;
      double time = 0;

      for (size_t i = 0; i < path.size(); i++) {
        double size_lat = 1.0 / scaling_y;
        double size_lon = 1.0 / scaling_x;

        int origin_lat = std::max(static_cast<int> (path[i].first / scaling_y - size_lat / 2), 0);
        int origin_lon = std::max(static_cast<int> (path[i].second / scaling_x - size_lon / 2), 0);

        int top_right_lat = std::min(static_cast<int> (path[i].first / scaling_y + size_lat / 2), static_cast<int> (original_nc_data.n_rows) - 1);
        int top_right_lon = std::min(static_cast<int> (path[i].second / scaling_x + size_lon / 2), static_cast<int> (original_nc_data.n_cols) - 1);

        gp_model.plot(path[i].first, path[i].second, 0.1, cv::Scalar(0, 0, 255), -1);

        utils::nc_data_t sub_nc_data = utils::extractSubRegionNC(original_nc_data, origin_lat, origin_lon, top_right_lat, top_right_lon);

        info_path sub_way_points = recursive(sub_nc_data, stages, current_layer + 1, i, mi_value, time);
        total_mi_value += mi_value;
        total_time += time;

        for (auto &p : sub_way_points) {
          p.first = p.first + origin_lat;
          p.second = p.second + origin_lon;
        }
        way_points.insert(std::end(way_points), std::begin(sub_way_points), std::end(sub_way_points));
      }

      gp_model.resize_image(cv::Size(original_nc_data.n_cols, original_nc_data.n_rows));
      gp_model.write(stages, current_layer, point_num);
    } else {
      for (auto &p : path) {
        //      std::cout << "recursive p " << p.first << " " << p.second << std::endl;
        gp_model.plot(p.first, p.second, 0.1, cv::Scalar(255, 0, 0), -1);
        p.first *= (original_nc_data.n_rows / grid_y_size);
        p.second *= (original_nc_data.n_cols / grid_x_size);
      }

      gp_model.resize_image(cv::Size(original_nc_data.n_cols, original_nc_data.n_rows));
      gp_model.write(stages, current_layer, point_num);

      return path;
    }

    return way_points;
  }

  void Info::setMdpMapSize(const size_t &lat_size, const size_t &lon_size) {
    mdp_lat_size = lat_size;
    mdp_lon_size = lon_size;

    // initial var map
    varMap.clear();
    varMap.resize(nc_data.n_rows * nc_data.n_cols);
    for (size_t i = 0; i < nc_data.n_rows; i++) {
      for (size_t j = 0; j < nc_data.n_cols; j++) {
        size_t cell[] = {i, j};
        varMap[i * nc_data.n_cols + j] = gp_model_var->var(cell);
      }
    }

    // TODO
    varMap = utils::resampling2D(varMap, nc_data.n_cols, nc_data.n_rows, mdp_lon_size / static_cast<double> (nc_data.n_cols), mdp_lat_size / static_cast<double> (nc_data.n_rows));

    //  std::cout << "varMap.size() " << varMap.size() << std::endl;
    //    for (int i = mdp_lat_size - 1; i >= 0; i--) {
    //      for (int j = 0; j < mdp_lon_size; j++) {
    //        std::cout << varMap[i * mdp_lon_size + j] << " ";
    //      }
    //      std::cout << endl;
    //    }
  }

  void Info::updateVarianceMap(double scale_x, double scale_y) {
    // transform
    //  std::cout << "scale_x " << scale_x << " scale_y " << scale_y << std::endl;
    //  std::cout << "nc_data.n_rows " << nc_data.n_rows << " nc_data.n_cols " << nc_data.n_cols << std::endl;
    //  std::cout << "scale_y * nc_data.n_rows " << scale_y * static_cast<double> (nc_data.n_rows) << " scale_x* nc_data.n_cols " << scale_x * static_cast<double> (nc_data.n_cols) << std::endl;
    gp_point goal(scale_y * static_cast<double> (nc_data.n_rows), scale_x * static_cast<double> (nc_data.n_cols));
    //  std::cout << "goal " << goal.first << " " << goal.second << std::endl;

    goal.first = std::min<size_t>(goal.first, nc_data.n_rows - 1);
    goal.second = std::min<size_t>(goal.second, nc_data.n_cols - 1);

    goal.first = std::max<size_t>(goal.first, 0);
    goal.second = std::max<size_t>(goal.second, 0);

    size_t x[] = {goal.first, goal.second};
    gp_model_var->add_pattern(x, 0);
    varMap.clear();
    varMap.resize(nc_data.n_rows * nc_data.n_cols);
    for (size_t i = 0; i < nc_data.n_rows; i++) {
      for (size_t j = 0; j < nc_data.n_cols; j++) {
        size_t cell[] = {i, j};
        varMap[i * nc_data.n_cols + j] = gp_model_var->var(cell);
      }
    }
    // TODO
    varMap = utils::resampling2D(varMap, nc_data.n_cols, nc_data.n_rows, mdp_lon_size / static_cast<double> (nc_data.n_cols), mdp_lat_size / static_cast<double> (nc_data.n_rows));

    //  std::cout << "varMap.size() " << varMap.size() << std::endl;
    //  for (int i = mdp_lat_size - 1; i >= 0; i--) {
    //    for (int j = 0; j < mdp_lon_size; j++) {
    //      std::cout << varMap[i * mdp_lon_size + j] << " ";
    //    }
    //    std::cout << endl;
    //  }
  }

  //void Info::updateVarianceMap() {
  //  std::map<std::pair<int, int>, std::pair<double, double>> previous_samples = gp_model_var->get_samples();
  //  gp_model_var->clear_samples();
  //
  //  for (auto &s : previous_samples) {
  //    int x[] = {s.first.first, s.first.second};
  //    //    std::cout << "s.second.second " << s.second.second << std::endl;
  //    gp_model_var->add_pattern_noise(x, s.second.first, s.second.second + 0.02); // time-variant
  //  }
  //
  //  varMap.clear();
  //  varMap.resize(nc_data.n_rows * nc_data.n_cols);
  //  for (int i = 0; i < nc_data.n_rows; i++) {
  //    for (int j = 0; j < nc_data.n_cols; j++) {
  //      Eigen::VectorXi cell(2);
  //      cell << i, j;
  //      varMap[i * nc_data.n_cols + j] = gp_model_var->var(cell);
  //    }
  //  }
  //
  //  // TODO
  //  varMap = utils::resampling2D(varMap, nc_data.n_cols, nc_data.n_rows, mdp_lon_size / static_cast<double> (nc_data.n_cols), mdp_lat_size / static_cast<double> (nc_data.n_rows));
  //
  //  //  std::cout << "varMap.size() " << varMap.size() << std::endl;
  //  //  std::cout.precision(1);
  //  //  for (int i = nc_data.n_rows - 1; i >= 0; i--) {
  //  //    for (int j = 0; j < nc_data.n_cols; j++) {
  //  //      std::cout << varMap[i * nc_data.n_cols + j] << " ";
  //  //    }
  //  //    std::cout << endl;
  //  //  }
  //
  //}

  void Info::WriteLengthResult(const std::vector<std::pair<size_t, size_t>> &path, const size_t &stages) {
    std::ofstream length_file(dir_name + "/" + std::to_string(stages) + "stages_" + std::to_string(observations.size()) + "obs_" + solver->get_name() + "_length_result.txt");
    double length = 0;
    for (int i = 0; i < path.size(); i++) {

      if (i != path.size() - 1) {
        int x1 = path[i].first;
        int y1 = path[i].second;

        int x2 = path[i + 1].first;
        int y2 = path[i + 1].second;
        length += std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
      } else {
        int x1 = path[i].first;
        int y1 = path[i].second;

        int x2 = path[0].first;
        int y2 = path[0].second;
        length += std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
      }
    }

    std::cout << "length " << length << std::endl;
    length_file << length << std::endl;
    length_file.close();
  }

  void Info::WriteTotalMutualInformation(const double &mi_value, const size_t &stages) {
    std::ofstream mutual_information_file(dir_name + "/" + std::to_string(stages) + "stages_" + std::to_string(observations.size()) + "obs_" + solver->get_name() + "_mutual_information.txt");
    mutual_information_file << std::fixed << mi_value << std::endl;
    std::cout << std::fixed << "mi_value " << mi_value << std::endl;
    mutual_information_file.close();
  }

  void Info::WriteTimeResult(const double &time, const size_t &stages, const size_t &count) {
    std::ofstream time_file(dir_name + "/" + std::to_string(stages) + "stages_" + std::to_string(observations.size()) + "obs_" + solver->get_name() + "_time" + std::to_string(count) + ".txt");
    time_file << time << std::endl;
    //  std::cout << "Waypoints time " << time << std::endl;
    time_file.close();
  }

  void Info::WritePathTimeResult(const double &time, const size_t &stages, const size_t &count) {
    std::ofstream time_file(dir_name + "/" + std::to_string(num_robots) + "robots_" + std::to_string(rounds) + "rounds_" + std::to_string(stages) + "stages_" + std::to_string(observations.size()) + "obs" + "_time" + std::to_string(count) + ".txt");
    time_file << time << std::endl;
    //  std::cout << "Path time " << time << std::endl;
    time_file.close();
  }

  const std::vector<double> &Info::getVarMap() {
    return varMap;
  }

  void Info::drawBackGround(const size_t &frame_count) {  
    for (int i = 0; i < nc_data.n_rows; i++) {
      for (int j = 0; j < nc_data.n_cols; j++) {
        double y = nc_data.frames[frame_count].cells[i][j].salinity;
        if (std::isnan(y)) {
          continue;
        } 
        double value = 255.0 - (y - nc_data.min) / nc_data.range * 255.0;
        if (value < 0) {
          value = 0;
        } else if (value > 255.0) {
          value = 255.0;
        }
        image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(value, value, value);
      }
    }

    for (const auto &o : observations) {
      cv::circle(image, cv::Point(o.second, nc_data.n_rows - o.first - 1), 3, cv::Scalar(0, 255, 255), -1);
//      image.at<cv::Vec3b>(nc_data.n_rows - o.first - 1, o.second) = cv::Vec3b(0.0, 255.0, 255.0);
    }
  }

  void Info::drawPath(const info_path &path) {
    for (size_t i = 0; i < path.size(); i++) {
//      std::cout << path[i].first << " " << path[i].second << std::endl;
      if (i == 0) {
        cv::circle(image, cv::Point(path[i].second, nc_data.n_rows - path[i].first - 1), 12, cv::Scalar(0, 0, 255), -1);
      } else {
        cv::circle(image, cv::Point(path[i].second, nc_data.n_rows - path[i].first - 1), 12, cv::Scalar(255, 0, 0), -1);
      }
      if (i < path.size() - 1) {
        cv::Point point1(path[i].second, nc_data.n_rows - path[i].first - 1);
        cv::Point point2(path[i + 1].second, nc_data.n_rows - path[i + 1].first - 1);
        cv::line(image, point1, point2, cv::Scalar(0, 255, 0), 4);
      } else {
//        cv::Point point1(path[i].second, nc_data.n_rows - path[i].first - 1);
//        cv::Point point2(path[0].second, nc_data.n_rows - path[0].first - 1);
//        cv::line(image, point1, point2, cv::Scalar(255, 0, 255));
      }
    }
  }

  info_path Info::tsp_tour(const info_path &final_way_points) {
    info_path final_shortest_path;

    std::ofstream tsp_file(dir_name + "/tsp.txt");
    std::cout << "dir_name " << dir_name << std::endl;
    for (size_t i = 0; i < final_way_points.size(); i++) {
      tsp_file << 0 << " ";
      for (size_t j = 1; j < final_way_points.size(); j++) {
        int x1 = static_cast<int>(final_way_points[i].first);
        int y1 = static_cast<int>(final_way_points[i].second);

        int x2 = static_cast<int>(final_way_points[j].first);
        int y2 = static_cast<int>(final_way_points[j].second);
        tsp_file << std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)) << " ";
      }
      tsp_file << std::endl;
    }
    tsp_file.close();

    std::vector<int> shortest_path = tsp_path(dir_name + "/tsp.txt");

    for (size_t i = 0; i < shortest_path.size(); i++) {
      std::cout << "shortest_path[i] " << shortest_path[i] << " final_way_points[shortest_path[i]] " << final_way_points[shortest_path[i]].first << " " << final_way_points[shortest_path[i]].second << std::endl;
      final_shortest_path.push_back(final_way_points[shortest_path[i]]);
    }

    return final_shortest_path;
  }
}