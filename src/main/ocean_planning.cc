#include <iostream>

#include <boost/filesystem.hpp>

#include <Eigen/Dense>

#include <sparse_online_gp.h>

#include "utils/data_loader.h"
#include "info_methods/info_core.h"
#include "info_methods/gp_model.h"
#include "info_methods/gp_model_old.h"
//#include "info_methods/dp_solver.h"
#include "info_methods/random_solver.h"

extern std::string dir_name;
const size_t gp_capacity = 100;

info_planner::info_path samplingPoints(info_planner::gp_point start, info_planner::gp_point end)
{
  info_planner::info_path sampling_points;

  int x0 = static_cast<int> (start.first);
  int y0 = static_cast<int> (start.second);

  int x1 = static_cast<int> (end.first);
  int y1 = static_cast<int> (end.second);

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  while (true)
  {
    sampling_points.push_back(info_planner::gp_point(x0, y0));
    if (x0 == x1 && y0 == y1)
    {
      return sampling_points;
    }
    int e2 = (err << 1);
    if (e2 > -dy)
    {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y0 += sy;
    }
  }
  return sampling_points;
}

void drawImage(info_planner::GP_Model gp_model, const utils::nc_data_t &nc_data, const size_t &frame_count, const size_t &replanning_count, const bool &stop)
{
  cv::Mat sogp_image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(0, 0, 0));
  std::vector<double> sogp_f(nc_data.n_rows * nc_data.n_cols, 0);
  std::vector<double> sogp_var(nc_data.n_rows * nc_data.n_cols, 0);

  int count = 0;
  for (size_t i = 0; i < nc_data.n_rows; i++)
  {
    for (size_t j = 0; j < nc_data.n_cols; j++)
    {
      double y = nc_data.frames[frame_count].cells[i][j].salinity;
      // exclude invalid points
      if (std::isnan(y))
      {
        continue;
      }
      size_t x[] = {i, j};
      //      double sogp_f = gp_model.f(x);
      sogp_f[i * nc_data.n_cols + j] = gp_model.f(x);
      sogp_var[i * nc_data.n_cols + j] = gp_model.var(x);
      count++;

      //      max = std::max(sogp_f[i * nc_data.n_cols + j], max);
      //      min = std::min(sogp_f[i * nc_data.n_cols + j], min);
      //            std::cout << "sogp_f " << sogp_f << std::endl;

      //      sogp_image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b((35 - sogp_f) / 5 * 255.0, (35 - sogp_f) / 5 * 255.0, (35 - sogp_f) / 5 * 255.0);
    }
  }

  std::cout << "max " << nc_data.max << std::endl;
  std::cout << "min " << nc_data.min << std::endl;
  std::cout << "range " << nc_data.range << std::endl;

  std::ofstream mean_file(dir_name + "/sogp_" + std::to_string(replanning_count) + "_mean.txt");
  std::ofstream var_file(dir_name + "/sogp_" + std::to_string(replanning_count) + "_var.txt");

  for (size_t i = 0; i < nc_data.n_rows; i++)
  {
    for (size_t j = 0; j < nc_data.n_cols; j++)
    {
      double y = nc_data.frames[frame_count].cells[i][j].salinity;

      // exclude invalid points
      if (std::isnan(y))
      {
        mean_file << std::numeric_limits<double>::max() << std::endl;
        var_file << std::numeric_limits<double>::max() << std::endl;
        continue;
      }
      else
      {
        mean_file << sogp_f[i * nc_data.n_cols + j] << std::endl;
        var_file << sogp_var[i * nc_data.n_cols + j] << std::endl;
      }

      double value = 255.0;
      if (nc_data.range != 0)
      {
        value = 255.0 - (sogp_f[i * nc_data.n_cols + j] - nc_data.min) / nc_data.range * 255.0;
      }
      if (value < 0)
      {
        value = 0;
      }
      else if (value > 255.0)
      {
        value = 255.0;
      }
      //      std::cout << "sogp_f[i * nc_data.n_cols + j] " << sogp_f[i * nc_data.n_cols + j] << std::endl;
      sogp_image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(value, value, value);
    }
  }
  mean_file.close();
  var_file.close();
  cv::imwrite(dir_name + "/sogp_" + std::to_string(frame_count) + "_replanning_count" + std::to_string(replanning_count) + "_stop" + std::to_string(stop) + ".png", sogp_image);
}

void experiment(const utils::nc_data_t &nc_data, const std::string &dir_name, const double &threshold, const info_planner::gp_point &starting_point, const size_t &stopping_threshold)
{
  std::string config_file("../configs/config.yaml");
  utils::Parameters::Ptr pParams = utils::Parameters::Ptr(new utils::Parameters(config_file));

  // Initial hyperparameters
  std::vector<double> cov_params;
  cov_params.push_back(pParams->getParamNode()["info_plan"]["gp_params"]["cov_params1"].as<double>());
  cov_params.push_back(pParams->getParamNode()["info_plan"]["gp_params"]["cov_params2"].as<double>());
  cov_params.push_back(pParams->getParamNode()["info_plan"]["gp_params"]["cov_params3"].as<double>());
  cov_params.push_back(pParams->getParamNode()["info_plan"]["gp_params"]["cov_params4"].as<double>());

  const size_t stages = pParams->getParamNode()["info_plan"]["stages"].as<size_t>();
  const size_t layers = 1;
  const size_t grid_x_size = pParams->getParamNode()["info_plan"]["grid_x_size"].as<size_t>();
  const size_t grid_y_size = pParams->getParamNode()["info_plan"]["grid_y_size"].as<size_t>();

  size_t frame_count = 0;
  std::cout << "current frame_count: " << frame_count << std::endl;

  for (size_t frame_count = 0; frame_count < nc_data.frames.size(); frame_count++)
  {
    cv::Mat truth_image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(0, 0, 0));
    std::ofstream data_file(dir_name + "/frame_count" + std::to_string(frame_count) + "_truth.txt");
    for (size_t i = 0; i < nc_data.n_rows; i++)
    {
      for (size_t j = 0; j < nc_data.n_cols; j++)
      {
        double truth_y = nc_data.frames[frame_count].cells[i][j].salinity;

        // exclude invalid points
        if (std::isnan(truth_y))
        {
          data_file << std::numeric_limits<double>::max() << std::endl;
          continue;
        }
        else
        {
          data_file << truth_y << std::endl;
        }

        double value = 255.0 - (truth_y - nc_data.min) / nc_data.range * 255.0;
        if (value < 0)
        {
          value = 0;
        }
        else if (value > 255.0)
        {
          value = 255.0;
        }

        truth_image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(value, value, value);
      }
    }
    data_file.close();
    cv::imwrite(dir_name + "/frame_count" + std::to_string(frame_count) + "_truth.png", truth_image);
  }

  // Initial observations: empty
  std::set<info_planner::gp_point> init_observations;

  // Initial starting_points
  std::vector<info_planner::gp_point> starting_points(1);
  starting_points[0] = starting_point;
  info_planner::gp_point current_position = starting_points[0];

  // Initial gp_model
//  info_planner::GP_Model gp_model(nc_data, "dp", 2, "CovSum(CovSEard, CovNoise)", cov_params, gp_capacity, 1e-20, init_observations, true);
    info_planner::GP_Model gp_model(nc_data, "random", 2, "CovSum(CovSEard, CovNoise)", cov_params, gp_capacity, 1e-20, init_observations, true);

//  info_planner::Solver_Interface *solver = new info_planner::DP_Solver();
    info_planner::Solver_Interface *solver = new info_planner::Random_Solver();

  size_t replanning_count = 0;
  const size_t counter_threshold = gp_capacity - static_cast<size_t> (static_cast<double> (gp_capacity) * threshold);
  size_t total_samples = 0;
  std::ofstream mse_file(dir_name + "/mse.txt");
  std::ofstream truth_mse_file(dir_name + "/truth_mse.txt");

  size_t old_sample_counter = 0;
  bool stop = true;

  drawImage(gp_model, nc_data, frame_count, total_samples, stop);
  while (true)
  {
    // Add observations
    std::set<info_planner::gp_point> observations;
    std::cout << "gp_model.get_samples().size() " << gp_model.get_samples().size() << std::endl;
    for (const auto& s : gp_model.get_samples())
    {
      observations.insert(s.first);
    }

    info_planner::Info::Ptr pInfo = info_planner::Info::Ptr(new info_planner::Info(pParams, cov_params, layers, grid_x_size, grid_y_size, nc_data, starting_points, observations, solver));
    pInfo->num_robots = 1;
    info_planner::info_paths paths = pInfo->run(nc_data, frame_count, stages, replanning_count);

    paths[0].push_back(starting_points[0]);

    int start_x = static_cast<int> (paths[0][0].first);
    int start_y = static_cast<int> (paths[0][0].second);

    int x1 = static_cast<int> (paths[0][1].first);
    int y1 = static_cast<int> (paths[0][1].second);

    int x2 = static_cast<int> (paths[0][paths[0].size() - 2].first);
    int y2 = static_cast<int> (paths[0][paths[0].size() - 2].second);

    info_planner::info_path final_path;
    if (std::sqrt(std::pow(start_x - x1, 2) + std::pow(start_y - y1, 2)) < std::sqrt(std::pow(start_x - x2, 2) + std::pow(start_y - y2, 2)))
    {
      for (size_t i = 0; i < paths[0].size() - 1; i++)
      {
        std::cout << paths[0][i].first << " " << paths[0][i].second << std::endl;
        final_path.push_back(paths[0][i]);
      }
    }
    else
    {
      for (size_t i = paths[0].size() - 1; i >= 1; i--)
      {
        std::cout << paths[0][i].first << " " << paths[0][i].second << std::endl;
        final_path.push_back(paths[0][i]);
      }
    }

    // if optimization was done, reset the counter
    if (stop)
    {
      if (replanning_count == 0)
      { // special case for the first because there's no sample at all
        old_sample_counter = gp_capacity;
      }
      else
      {
        old_sample_counter = observations.size();
      }
      stop = false;
    }
    // samplingPoints
    for (size_t i = 0; i < final_path.size() - 1; i++)
    {
      std::cout << "sampling from " << final_path[i].first << " " << final_path[i].second << " to " << final_path[i + 1].first << " " << final_path[i + 1].second << std::endl;
      info_planner::info_path points = samplingPoints(final_path[i], final_path[i + 1]);
      for (size_t j = 0; j < points.size(); j++)
      {
        current_position.first = points[j].first;
        current_position.second = points[j].second;
        starting_points[0] = current_position;

        size_t x[] = {current_position.first, current_position.second};
        double y = nc_data.frames[frame_count].cells[x[0]][x[1]].salinity;
        if (std::isnan(y))
        {
          continue; // invalid sampling point
        }
        total_samples++;
        if (total_samples % (stopping_threshold / (nc_data.frames.size())) == 0)
        { // change the nc_data frame
          frame_count++;
          if (frame_count == nc_data.frames.size())
          {
            frame_count = nc_data.frames.size() - 1;
          }
          std::cout << "current frame_count: " << frame_count << std::endl;
        }
        //        std::cout << "sample " << points[j].first << " " << points[j].second << " ";
        //        truth_image.at<cv::Vec3b>(nc_data.n_rows - current_position.first - 1, current_position.second) = cv::Vec3b(255.0, 0, 0);
        int return_code = gp_model.add_pattern(x, y);

        if (return_code >= 0 && return_code < old_sample_counter)
        {
          old_sample_counter--;
        }
        if (total_samples % 50 == 0)
        {
          // drawImage(gp_model, nc_data, total_samples, stop);
          pInfo->draw(current_position, gp_model.get_samples(), total_samples, frame_count, i);
        }

        // calculate pred error
        // total squared error
        double sogp_tss = 0;
        double truth_tss = 0;
        size_t prev_frame_count = 0;
        if (frame_count > 0) {
          prev_frame_count = frame_count - 1;
        }
        for (size_t i = 0; i < nc_data.n_rows; i++)
        {
          for (size_t j = 0; j < nc_data.n_cols; j++)
          {
            double prev_y = nc_data.frames[prev_frame_count].cells[i][j].salinity;
            double y = nc_data.frames[frame_count].cells[i][j].salinity;
            // exclude invalid points
            if (std::isnan(y))
            {
              continue;
            }
            size_t x[] = {i, j};
            double sogp_f = gp_model.f(x);

            double sogp_error = sogp_f - y;
            double truth_error = prev_y - y;
            sogp_tss += sogp_error * sogp_error;
            truth_tss += truth_error * truth_error;
          }
        }
        //        std::cout << "sogp mse = " << sogp_tss / (nc_data.n_rows * nc_data.n_cols) << " total_samples " << total_samples << std::endl;
        mse_file << sogp_tss / (nc_data.n_rows * nc_data.n_cols) << std::endl;
        truth_mse_file << truth_tss / (nc_data.n_rows * nc_data.n_cols) << std::endl;

        //        std::cout << " return_code " << return_code;
        //        std::cout << " old_sample_counter " << old_sample_counter << std::endl;
        //        std::cout << std::endl;
        // first time replaning is triggerd when the set is full
        if (old_sample_counter <= counter_threshold || total_samples >= stopping_threshold)
        {
          stop = true;
          break; // optimization, reset hyper parameters, replanning
        }
      }
      if (stop)
      {
        break;
      }
    }
    if (total_samples >= stopping_threshold)
    {
      break;
    }
    std::cout << "stop " << stop << std::endl;
    std::cout << "replanning " << std::endl;
    replanning_count++;

    if (stop)
    {
      // offline optimization
      std::cout << "old hypers" << std::endl;
      std::cout << gp_model.get_loghyper() << std::endl;
      BGD bgd(0.0005, 1e-6, 300);
      gp_model.training(bgd);
      std::cout << "new hypers" << std::endl;
      std::cout << gp_model.get_loghyper() << std::endl;
      Eigen::VectorXd new_params = gp_model.get_loghyper();
      for (size_t k = 0; k < cov_params.size(); k++)
      {
        cov_params[k] = new_params(k);
      }
      //      drawImage(gp_model, nc_data, total_samples, stop);
    }
    drawImage(gp_model, nc_data, frame_count, total_samples, stop);

    //    if (replanning_count == 40) {
    //      break;
    //    }

  }
  mse_file.close();
  truth_mse_file.close();
  delete solver;
}

int main(int argc, char *argv[])
{
  std::string config_file("../configs/config.yaml");
  utils::Parameters::Ptr pParams = utils::Parameters::Ptr(new utils::Parameters(config_file));
  utils::DataLoader::Ptr pData = utils::DataLoader::Ptr(new utils::DataLoader(pParams));
  pData->importVecFieldData();
  utils::nc_data_t nc_data = pData->getRawNCdata();

  double max = -10000;
  double min = 10000;

  for (size_t frame_count = 0; frame_count < nc_data.frames.size(); frame_count++)
  {
    for (size_t i = 0; i < nc_data.n_rows; i++)
    {
      for (size_t j = 0; j < nc_data.n_cols; j++)
      {
        double truth_y = nc_data.frames[frame_count].cells[i][j].salinity;
        // exclude invalid points
        if (std::isnan(truth_y))
        {
          continue;
        }
        max = std::max(truth_y, max);
        min = std::min(truth_y, min);
      }
    }
  }

  nc_data.max = max;
  nc_data.min = min;
  nc_data.range = max - min;
  std::cout << "max " << max << std::endl;
  std::cout << "min " << min << std::endl;
  std::cout << "nc_data.range " << nc_data.range << std::endl;

  // Compare mean squared error between first frame and other frames
  for (size_t frame_count = 0; frame_count < nc_data.frames.size(); frame_count++) 
  {
    double truth_tss = 0;
    for (size_t i = 0; i < nc_data.n_rows; i++)
    {
      for (size_t j = 0; j < nc_data.n_cols; j++)
      {
        double truth_0 = nc_data.frames[0].cells[i][j].salinity;
        double truth = nc_data.frames[frame_count].cells[i][j].salinity;
        // exclude invalid points
        if (std::isnan(truth_0) || std::isnan(truth))
        {
          continue;
        }
        double truth_error = truth_0 - truth;
        truth_tss += truth_error * truth_error;
      }
    }
    std::cout << "frame_count " << frame_count << " mse: " << truth_tss / (nc_data.n_rows * nc_data.n_cols) << std::endl;
  }
  
  std::vector<info_planner::gp_point> starting_points;
  //  srand(1); // same starting_locations for different threshold
  //  for (int i = 0; i < 10; i++) {
  //    size_t row = rand() % nc_data.n_rows;
  //    size_t col = rand() % nc_data.n_cols;
  //    double y = nc_data.frames[0].cells[row][col].salinity;
  //    if (std::isnan(y)) {
  //      i--;
  //      continue;
  //    }
  //    starting_points.push_back(info_planner::gp_point(row, col));
  //    std::cout << row << " " << col << std::endl;
  //  }

  std::vector<size_t> stopping_thresholds;

  //  starting_points.push_back(info_planner::gp_point(2, 276));
  //  stopping_thresholds.push_back(2000);
  //
  //  starting_points.push_back(info_planner::gp_point(213, 160));
  //  stopping_thresholds.push_back(2000);

  starting_points.push_back(info_planner::gp_point(236, 79));
  stopping_thresholds.push_back(2000);

  //  starting_points.push_back(info_planner::gp_point(37, 120));
  //  stopping_thresholds.push_back(2000);
  //
  //  starting_points.push_back(info_planner::gp_point(11, 280));
  //  stopping_thresholds.push_back(2000);
  //
  //  starting_points.push_back(info_planner::gp_point(261, 37));
  //  stopping_thresholds.push_back(2000);
  //
  //  starting_points.push_back(info_planner::gp_point(76, 211));
  //  stopping_thresholds.push_back(2000);
  //
  //  starting_points.push_back(info_planner::gp_point(68, 207));
  //  stopping_thresholds.push_back(2000);
  //
  //  starting_points.push_back(info_planner::gp_point(40, 30));
  //  stopping_thresholds.push_back(2000);
  //
  //  starting_points.push_back(info_planner::gp_point(189, 31));
  //  stopping_thresholds.push_back(2000);

  // 0.1~1.0
  //  const double threshold = std::stod(std::string(argv[1]));
  //  const double i = std::stoi(std::string(argv[2]));
  for (double threshold = 0.6; threshold <= 0.6; threshold += 0.1)
  {
    for (int i = 0; i < starting_points.size(); i++)
    {
      size_t row = starting_points[i].first;
      size_t col = starting_points[i].second;
      dir_name = "results" + std::to_string(row) + std::to_string(col) + "_threshold" + std::to_string(static_cast<size_t> (threshold * gp_capacity));
      if (nc_data.frames.size() > 1) 
      {
        dir_name += "_" + std::to_string(nc_data.frames.size()) + "frames";
      }
      if (boost::filesystem::create_directory(dir_name))
      {
        std::cout << "Success" << "\n";
      }
      experiment(nc_data, dir_name, threshold, info_planner::gp_point(row, col), stopping_thresholds[i]);
    }
  }

  return EXIT_SUCCESS;
}
