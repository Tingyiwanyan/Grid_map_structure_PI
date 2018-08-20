#include <iostream>

#include <Eigen/Dense>

#include <sparse_online_gp.h>

#include "utils/data_loader.h"
#include "info_methods/gp_model.h"
#include "info_methods/gp_model_old.h"
#include "info_methods/solver_interface.h"

info_planner::info_path samplingPoints(info_planner::gp_point start, info_planner::gp_point end) {
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

  while (true) {
    sampling_points.push_back(info_planner::gp_point(x0, y0));
    if (x0 == x1 && y0 == y1) {
      return sampling_points;
    }
    int e2 = (err << 1);
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
  return sampling_points;
}

double convert_to_gray_scale(const double &f, const double &max, const double &min) {
  const double range = max - min;
  double value = 255.0 - (f - min) / range * 255.0;
  if (value < 0) {
    value = 0;
  } else if (value > 255.0) {
    value = 255.0;
  }
  return value;
}

int main(int argc, char *argv[]) {
  std::string config_file("../configs/config.yaml");
  utils::Parameters::Ptr pParams = utils::Parameters::Ptr(new utils::Parameters(config_file));
  utils::DataLoader::Ptr pData = utils::DataLoader::Ptr(new utils::DataLoader(pParams));
  pData->importVecFieldData();
  utils::nc_data_t nc_data = pData->getRawNCdata();

  std::vector<double> cov_params;
  cov_params.push_back(1.0);
  cov_params.push_back(1.0);
  cov_params.push_back(1.0);
  cov_params.push_back(-1.0);

  const int n = atoi(argv[1]); // n training samples
  //  const int m = atoi(argv[2]); // m testing samples

  std::set<info_planner::gp_point> observations;
  srand(1);
  size_t i = 0;
  while (i < n) {
    size_t row = rand() % nc_data.n_rows;
    size_t col = rand() % nc_data.n_cols;
    double y = nc_data.frames[0].cells[row][col].salinity;
//    std::cout << "row " << row << " col " << col << std::endl;
    if (std::isnan(y)) {
      continue;
    }
    observations.insert(info_planner::gp_point(row, col));
    i++;
  }
  
//  info_planner::info_path samples = samplingPoints(info_planner::gp_point(196, 176), info_planner::gp_point(192, 87));
//  for (size_t i = 0; i < samples.size(); i++) {
//    size_t row = samples[i].first;
//    size_t col = samples[i].second;
//    double y = nc_data.frames[0].cells[row][col].salinity;
//
//    if (std::isnan(y)) {
//      continue;
//    }
//    observations.insert(info_planner::gp_point(row, col));
//  }
//  samples = samplingPoints(info_planner::gp_point(192, 87), info_planner::gp_point(32, 87));
//  for (size_t i = 0; i < samples.size(); i++) {
//    size_t row = samples[i].first;
//    size_t col = samples[i].second;
//    double y = nc_data.frames[0].cells[row][col].salinity;
//
//    if (std::isnan(y)) {
//      continue;
//    }
//    observations.insert(info_planner::gp_point(row, col));
//  }


//  observations.insert(info_planner::gp_point(0, nc_data.n_cols - 1));
//  std::cout << nc_data.frames[0].cells[0][nc_data.n_cols - 1].salinity << std::endl;
//  observations.insert(info_planner::gp_point(nc_data.n_rows - 1, 0));
//  std::cout << nc_data.frames[0].cells[nc_data.n_rows - 1][0].salinity << std::endl;
  
  info_planner::GP_Model gp_model(nc_data, "dp", 2, "CovSum(CovSEard, CovNoise)", cov_params, 300, 1e-20, observations, true);
  GP_Model_Old gp_model_old_manual(nc_data, "dp", 2, "CovSum(CovSEard, CovNoise)", cov_params, observations, true);
  GP_Model_Old gp_model_old_auto(nc_data, "dp", 2, "CovSum(CovSEard, CovNoise)", cov_params, observations, true);
  
//  size_t map_x[] = {0, 0};
//  gp_model.add_pattern(map_x, nc_data.frames[0].cells[0][0].salinity);
//  std::cout << gp_model.get_samples().size() << std::endl;
//  gp_model.add_pattern(map_x, nc_data.frames[0].cells[0][0].salinity);
//  std::cout << gp_model.get_samples().size() << std::endl;
//  gp_model.add_pattern(map_x, nc_data.frames[0].cells[0][0].salinity);
//  std::cout << gp_model.get_samples().size() << std::endl;
//  
//  gp_model_old_manual.add_pattern(map_x, nc_data.frames[0].cells[0][0].salinity);
//  std::cout << gp_model_old_manual.get_samples().size() << std::endl;
//  gp_model_old_manual.add_pattern(map_x, nc_data.frames[0].cells[0][0].salinity);
//  std::cout << gp_model_old_manual.get_samples().size() << std::endl;
//  gp_model_old_manual.add_pattern(map_x, nc_data.frames[0].cells[0][0].salinity);
//  std::cout << gp_model_old_manual.get_samples().size() << std::endl;

  Eigen::VectorXd sogp_params = gp_model.get_loghyper();
  Eigen::VectorXd gp_manual_params = gp_model_old_manual.get_loghyper();
  Eigen::VectorXd gp_auto_params = gp_model_old_auto.get_loghyper();
  // offline optimization
  //  BGD bgd(0.0005, 1e-6, 300);
  //  gp_model.training(bgd);

  gp_model_old_auto.training();

  Eigen::VectorXd sogp_hypers = gp_model.get_loghyper();
  Eigen::VectorXd gp_manual_hypers = gp_model_old_manual.get_loghyper();
  Eigen::VectorXd gp_auto_hypers = gp_model_old_auto.get_loghyper();
  std::cout << "sogp_params" << std::endl;
  std::cout << sogp_params << std::endl;
  std::cout << "sogp_hypers" << std::endl;
  std::cout << sogp_hypers << std::endl;

  std::cout << "gp_manual_params" << std::endl;
  std::cout << gp_manual_params << std::endl;
  std::cout << "gp_manual_hypers" << std::endl;
  std::cout << gp_manual_hypers << std::endl;

  std::cout << "gp_auto_params" << std::endl;
  std::cout << gp_auto_params << std::endl;
  std::cout << "gp_auto_hypers" << std::endl;
  std::cout << gp_auto_hypers << std::endl;

  cv::Mat truth_image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat gp_manual_image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat gp_auto_image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat sogp_image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(0, 0, 0));
    
  // max min of ground truth
  std::cout << "max " << nc_data.max << std::endl;
  std::cout << "min " << nc_data.min << std::endl;
  std::cout << "range " << nc_data.range << std::endl;
  //
  
  // total squared error
  double gp_manual_tss = 0;
  double gp_auto_tss = 0;
  double sogp_tss = 0;
  for (size_t i = 0; i < nc_data.n_rows; i++) {
    for (size_t j = 0; j < nc_data.n_cols; j++) {
      double y = nc_data.frames[0].cells[i][j].salinity;
      // exclude invalid points
      if (std::isnan(y)) {
        continue;
      }

      size_t x[] = {i, j};
      double sogp_f = gp_model.f(x);
      double gp_manual_f = gp_model_old_manual.f(x);
      double gp_auto_f = gp_model_old_auto.f(x);
      
      double truth_value = convert_to_gray_scale(y, nc_data.max, nc_data.min);
      double sogp_value = convert_to_gray_scale(sogp_f, nc_data.max, nc_data.min);
      double gp_manual_value = convert_to_gray_scale(gp_manual_f, nc_data.max, nc_data.min);
      double gp_auto_value = convert_to_gray_scale(gp_auto_f, nc_data.max, nc_data.min);
              
      truth_image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(truth_value, truth_value, truth_value);
      gp_manual_image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(gp_manual_value, gp_manual_value, gp_manual_value);
      gp_auto_image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(gp_auto_value, gp_auto_value, gp_auto_value);
      sogp_image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(sogp_value, sogp_value, sogp_value);
      
      // exclude the training set
//      if (observations.find(info_planner::gp_point(i, j)) != observations.end()) {
//        continue;
//      }

      double sogp_error = sogp_f - y;
      sogp_tss += sogp_error*sogp_error;

      double gp_manual_error = gp_manual_f - y;
      gp_manual_tss += gp_manual_error*gp_manual_error;

      double gp_auto_error = gp_auto_f - y;
      gp_auto_tss += gp_auto_error*gp_auto_error;

    }
  }
  
  for (const auto &o : observations) {
    gp_manual_image.at<cv::Vec3b>(nc_data.n_rows - o.first - 1, o.second) = cv::Vec3b(255, 0, 0);
  }
  
  std::cout << "gp_manual mse = " << gp_manual_tss / (nc_data.n_rows * nc_data.n_cols) << std::endl;
  std::cout << "gp_auto mse = " << gp_auto_tss / (nc_data.n_rows * nc_data.n_cols) << std::endl;
  std::cout << "sogp mse = " << sogp_tss / (nc_data.n_rows * nc_data.n_cols) << std::endl;

  info_planner::info_path way_points;

//  info_planner::info_path path_points = samplingPoints(info_planner::gp_point(0, 0), info_planner::gp_point(100, 200));
//  for (size_t i = 0; i < path_points.size(); i++) {
//    cv::circle(truth_image, cv::Point(path_points[i].second, nc_data.n_rows - path_points[i].first - 1), 1, cv::Scalar(255, 0, 0), -1);
//  }
  
  cv::imwrite("results/truth.png", truth_image);
  cv::imwrite("results/gp_manual_" + std::to_string(observations.size()) + "obs" + ".png", gp_manual_image);
  cv::imwrite("results/gp_auto_" + std::to_string(observations.size()) + "obs" + ".png", gp_auto_image);
  cv::imwrite("results/sogp_" + std::to_string(observations.size()) + "obs" + ".png", sogp_image);

  gp_model_old_manual.write(0, 0, 0);
  return EXIT_SUCCESS;
}
