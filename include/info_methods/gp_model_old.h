#ifndef GP_MODEL_OLD_H
#define GP_MODEL_OLD_H

#include <map>
#include <set>
#include <utility>
#include "../utils/nc_data.h"
#include "../geometry_utils/Vector2.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace geometry_utils;

class GP_Model_Old {
private:
  // initialize Gaussian process for 2-D input using the squared exponential covariance function
  libgp::GaussianProcess gp;
  std::map<std::pair<size_t, size_t>, std::pair<double, double>> samples; // x1 x2, y noise
  cv::Mat image;

  std::string file_name;

  std::set<std::pair<size_t, size_t>> observations;

  static const double entropy_limit;
  static const double variance_limit;

public:
  GP_Model_Old(const utils::nc_data_t &nc_data, const std::string &file_name, const size_t &input_dim, const std::string &cov_func, const std::vector<double> &cov_params, std::set<std::pair<size_t, size_t>> observations, const bool &add_samples);
  GP_Model_Old(GP_Model_Old& other);

  void add_pattern(const size_t map_x[], const double &y);

  Eigen::VectorXd get_loghyper();
  
  void training();
  
  const std::map<std::pair<size_t, size_t>, std::pair<double, double>> &get_samples() const {
    return samples;
  }

  const std::set<std::pair<size_t, size_t>>&get_point_set() const {
    return point_set;
  }

  const utils::nc_data_t &get_nc_data() const {
    return nc_data;
  }

  void clear_samples();

  double f(const size_t map_x[]);

  double var(const size_t map_x[]);

  double cov(const Eigen::VectorXi &map_x1, const Eigen::VectorXi &map_x2);

  void plot(const size_t &i, const size_t &j, const double &value);

  void plot(const size_t &i, const size_t &j, int radius, const cv::Scalar &color, int thickness = 1);

  void write(const size_t &n, const size_t &layer, const size_t &point_num);

  void resize_image(const cv::Size &size);

  std::set<std::pair<size_t, size_t>> point_set;
  const utils::nc_data_t nc_data;

  double MutualInformation(const Eigen::VectorXi &A, const Eigen::MatrixXi &B);
  double Entropy(const Eigen::VectorXi &A);

  GP_Model_Old& operator=(const GP_Model_Old& other);

};


#endif /* GP_MODEL_OLD_H */