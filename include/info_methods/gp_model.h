#ifndef GP_MODEL_H
#define GP_MODEL_H

#include <map>
#include <set>
#include "../utils/nc_data.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <sparse_online_gp.h>
#include <optimizer/bgd.h>

namespace info_planner {
  typedef std::pair<size_t, size_t> gp_point;

  class GP_Model {
  private:
    // initialize Gaussian process for 2-D input using the squared exponential covariance function
    libsogp::SparseOnlineGaussianProcess gp;
    std::vector<std::pair<gp_point, std::pair<double, double>>> samples; // x1 x2, y noise
    cv::Mat image;

    std::string file_name;

    std::set<gp_point> observations;

    static constexpr double entropy_limit = 1e+6;
    static constexpr double variance_limit = 1e+6;

  public:
    GP_Model(const utils::nc_data_t &nc_data, const std::string &file_name, const size_t &input_dim, const std::string &cov_func, const std::vector<double> &cov_params, const size_t &max_bv_size, const double &tolerance_error, std::set<gp_point> observations, const bool &add_samples);
    GP_Model(const GP_Model& other);

    int add_pattern(const size_t map_x[], const double &y); // return value max:not added, max - 1: added, other:replaced point
    
    void training(BGD bgd);
    void recompute();
    Eigen::VectorXd get_loghyper();

    const std::vector<std::pair<gp_point, std::pair<double, double>>> &get_samples() const {
      return samples;
    }

    const std::set<gp_point>&get_point_set() const {
      return point_set;
    }

    const utils::nc_data_t &get_nc_data() const {
      return nc_data;
    }

    void clear_samples();

    double f(const size_t map_x[]);

    double var(const size_t map_x[]);

    //  double cov(const Eigen::VectorXi &map_x1, const Eigen::VectorXi &map_x2);

    void plot(const size_t &i, const size_t &j, const double &value);

    void plot(const size_t &i, const size_t &j, int radius, const cv::Scalar &color, int thickness = 1);

    void write(const size_t &n, const size_t &layer, const size_t &point_num);

    void resize_image(const cv::Size &size);

    std::set<gp_point> point_set;
    const utils::nc_data_t nc_data;

    double MutualInformation(const Eigen::VectorXi &A, const Eigen::MatrixXi &B);
    double Entropy(const Eigen::VectorXi &A);

    GP_Model& operator=(const GP_Model& other);

  };
}

#endif /* GP_MODEL_H */

