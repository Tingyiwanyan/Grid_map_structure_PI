#include <optimizer/rprop.h>

#include "info_methods/gp_model_old.h"

extern std::string dir_name;

const double GP_Model_Old::entropy_limit = 1e+6;
const double GP_Model_Old::variance_limit = 1e+6;

GP_Model_Old::GP_Model_Old(const utils::nc_data_t &nc_data, const std::string &file_name, const size_t &input_dim, const std::string &cov_func, const std::vector<double> &cov_params, std::set<std::pair<size_t, size_t>> observations, const bool &add_samples)
: gp(input_dim, cov_func), nc_data(nc_data), file_name(file_name), observations(observations),
image(nc_data.n_rows, nc_data.n_cols, CV_8UC3, cv::Scalar(0, 0, 0)) {

  Eigen::VectorXd params(gp.covf().get_param_dim());
  for (size_t i = 0; i < gp.covf().get_param_dim(); i++) {
    params(i) = cov_params[i];
  }
  gp.covf().set_loghyper(params);

  std::cout << "GP_Model nc_data.n_rows " << nc_data.n_rows << std::endl;
  std::cout << "GP_Model nc_data.n_cols " << nc_data.n_cols << std::endl;

  // Draw background
  for (size_t i = 0; i < nc_data.n_rows; i++) {
    for (size_t j = 0; j < nc_data.n_cols; j++) {
      double y = nc_data.frames[0].cells[i][j].salinity;
      plot(i, j, y);
    }
  }

  // Set observed samples
  if (add_samples) {
    for (const auto &o : observations) {
      size_t map_x[] = {o.first, o.second};
      double y = nc_data.frames[0].cells[map_x[0]][map_x[1]].salinity;
      add_pattern(map_x, y);
      //      std::cout << "o " << o.first << " " << o.second << std::endl;
      plot(o.first, o.second, 0.1, cv::Scalar(0, 255, 255), -1);
    }
  }

  point_set.clear();
  for (size_t i = 0; i < nc_data.n_rows; i++) {
    for (size_t j = 0; j < nc_data.n_cols; j++) {
      if (std::find(observations.begin(), observations.end(), std::pair<size_t, size_t>(i, j)) == observations.end() && !std::isnan(nc_data.frames[0].cells[i][j].salinity)) {
        point_set.insert(std::pair<size_t, size_t>(i, j));
      }
    }
  }
}

GP_Model_Old::GP_Model_Old(GP_Model_Old& other) : gp(other.gp.get_input_dim(), other.gp.covf().to_string()), nc_data(other.nc_data), file_name(other.file_name), observations(other.observations), point_set(other.point_set) {
  gp.covf().set_loghyper(other.gp.covf().get_loghyper());

  for (const auto& s : other.get_samples()) {
    size_t map_x[] = {s.first.first, s.first.second};
    add_pattern(map_x, s.second.first);
  }
}

Eigen::VectorXd GP_Model_Old::get_loghyper() {
  return gp.covf().get_loghyper();
}

void GP_Model_Old::training() {
  libgp::RProp rprop;
  rprop.maximize(&gp, 100, 0);
}

void GP_Model_Old::add_pattern(const size_t map_x[], const double &y) {
  // TODO: adding duplicates??
  
  double coord[] = {nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.x(), nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.y()};
  gp.add_pattern(coord, y);

  samples.insert(std::make_pair(std::pair<size_t, size_t>(map_x[0], map_x[1]), std::make_pair(y, gp.covf().get_loghyper()[2])));
}

//void GP_Model_Old::add_pattern_noise(const int map_x[], const double &y, double noise) {
//  double coord[] = {nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.x(), nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.y()};
//  gp.add_pattern_noise(coord, y, noise);
//  
//  samples.insert(std::make_pair(std::pair<int, int>(map_x[0], map_x[1]), std::make_pair(y, noise)));
//}

void GP_Model_Old::clear_samples() {
  gp.clear_sampleset();
  samples.clear();
}

double GP_Model_Old::f(const size_t map_x[]) {
  double coord[] = {nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.x(), nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.y()};
  return gp.f(coord);
}

double GP_Model_Old::var(const size_t map_x[]) {
  double coord[] = {nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.x(), nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.y()};
  return gp.var(coord);
}

double GP_Model_Old::cov(const Eigen::VectorXi &map_x1, const Eigen::VectorXi &map_x2) {
  double coord1[] = {nc_data.frames[0].cells[map_x1[0]][map_x1[1]].coord.x(), nc_data.frames[0].cells[map_x1[0]][map_x1[1]].coord.y()};

  double coord2[] = {nc_data.frames[0].cells[map_x2[0]][map_x2[1]].coord.x(), nc_data.frames[0].cells[map_x2[0]][map_x2[1]].coord.y()};

  Eigen::VectorXd c1(2);
  c1[0] = coord1[0];
  c1[1] = coord1[1];

  Eigen::VectorXd c2(2);
  c2[0] = coord2[0];
  c2[1] = coord2[1];

  return gp.covf().get(c1, c2);
}

void GP_Model_Old::plot(const size_t &i, const size_t &j, const double &value) {
  double gray_value = 255.0 - (value - nc_data.min) / nc_data.range * 255.0;
  if (gray_value < 0) {
    gray_value = 0;
  } else if (value > 255.0) {
    gray_value = 255.0;
  }
    
  image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(gray_value, gray_value, gray_value);
}

void GP_Model_Old::plot(const size_t &i, const size_t &j, int radius, const cv::Scalar &color, int thickness) {
  cv::circle(image, cv::Point(j, image.rows - i - 1), radius, color, thickness);
}

void GP_Model_Old::write(const size_t &n, const size_t &layer, const size_t &point_num) {
  cv::imwrite(dir_name + "/" + file_name + "_" + std::to_string(n) + "_layer" + std::to_string(layer) + "_" + std::to_string(point_num) + "_" + std::to_string(observations.size()) + "obs" + ".png", image);
}

void GP_Model_Old::resize_image(const cv::Size &size) {
  resize(image, image, size);
}

double GP_Model_Old::Entropy(const Eigen::VectorXi &A) {
  size_t x[] = {static_cast<size_t> (A[0]), static_cast<size_t> (A[1])};
  double e = std::log(2 * var(x) * M_PI * std::exp(1)) / 2;
  //  std::cout << "e " << e << std::endl;

  if (std::isnan(e) || std::isinf(e)) {
    if (e > 0) {
      e = entropy_limit;
    } else {
      e = -entropy_limit;
    }
  }
  return e;
}

double GP_Model_Old::MutualInformation(const Eigen::VectorXi &A, const Eigen::MatrixXi &B) {
  GP_Model_Old temp_gp = *this;

  for (size_t i = 0; i < B.rows(); i++) {
    size_t map_x[] = {static_cast<size_t> (B(i, 0)), static_cast<size_t> (B(i, 1))};
    temp_gp.add_pattern(map_x, 0);
  }

  //  for (const auto &s : samples) {
  //    std::cout << "s " << s.first.first << " " << s.first.second << " " << s.second.first << " " << s.second.second << std::endl;
  //  }
  //
  //  for (const auto &s : temp_gp.samples) {
  //    std::cout << "temp_gp s " << s.first.first << " " << s.first.second << " " << s.second.first << " " << s.second.second << std::endl;
  //  }

  //  std::cout << "Entropy(A) " << Entropy(A) << std::endl;
  //  std::cout << "temp_gp.Entropy(A) " << temp_gp.Entropy(A) << std::endl;

  return Entropy(A) - temp_gp.Entropy(A);
}
