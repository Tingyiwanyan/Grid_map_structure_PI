#include "info_methods/gp_model.h"
extern std::string dir_name;

namespace info_planner {

  GP_Model::GP_Model(const utils::nc_data_t &nc_data, const std::string &file_name, const size_t &input_dim, const std::string &cov_func, const std::vector<double> &cov_params, const size_t &max_bv_size, const double &tolerance_error, std::set<gp_point> observations, const bool &add_samples)
  : gp(input_dim, cov_func, max_bv_size, tolerance_error), nc_data(nc_data), file_name(file_name), observations(observations),
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

    clear_samples();
    // Set observed samples
    if (add_samples) {
      for (const auto &o : observations) {
        size_t map_x[] = {o.first, o.second};
        double y = nc_data.frames[0].cells[map_x[0]][map_x[1]].salinity;
        add_pattern(map_x, y);
        image.at<cv::Vec3b>(nc_data.n_rows - o.first - 1, o.second) = cv::Vec3b(0, 255, 255);
      }
    }

    point_set.clear();
    for (size_t i = 0; i < nc_data.n_rows; i++) {
      for (size_t j = 0; j < nc_data.n_cols; j++) {
        if (std::find(observations.begin(), observations.end(), gp_point(i, j)) == observations.end() && !std::isnan(nc_data.frames[0].cells[i][j].salinity)) {
          point_set.insert(gp_point(i, j));
        }
      }
    }
  }

  GP_Model::GP_Model(const GP_Model& other) : gp(other.gp.get_input_dim(), other.gp.covf().to_string(), other.gp.get_max_bv_size(), other.gp.get_tolerance_error()), point_set(other.point_set), nc_data(other.nc_data), file_name(other.file_name), observations(other.observations) {
    gp.covf().set_loghyper(other.gp.covf().get_loghyper());
    for (const auto& s : other.get_samples()) {
      size_t map_x[] = {s.first.first, s.first.second};
      add_pattern(map_x, s.second.first);
    }
  }

  int GP_Model::add_pattern(const size_t map_x[], const double &y) {
    // TODO: adding duplicates??
    int return_code = -2;
    for (const auto &s : samples) {
      if (s.first.first == map_x[0] && s.first.second == map_x[1]) {
        std::cout << "map_x[0] " << map_x[0] << " " << map_x[1] << std::endl;
        return return_code;
      }
    }
    
    Eigen::VectorXd coord(2);
    coord << nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.x(), nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.y();
    return_code = gp.add_pattern(coord, y);
    if (return_code != -2) {
      samples.push_back(std::make_pair(gp_point(map_x[0], map_x[1]), std::make_pair(y, gp.covf().get_loghyper()[2])));
    }

    if (return_code >= 0) {
//      std::cout << "removed point " << (samples.begin() + return_code)->first.first << " " << (samples.begin() + return_code)->first.second;
      samples.erase(samples.begin() + return_code);
    } else if (return_code == -1) {
//      std::cout << "added ";
    } else {
//      std::cout << "not added ";
    }
    return return_code;
  }

  void GP_Model::training(BGD bgd) {
    bgd.maximize(gp);
  }

  void GP_Model::recompute() {
    gp.recompute();
  }

  Eigen::VectorXd GP_Model::get_loghyper() {
    return gp.covf().get_loghyper();
  }

  //void GP_Model::add_pattern_noise(const size_t map_x[], const double &y, double noise) {
  //  double coord[] = {nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.x(), nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.y()};
  //  gp.add_pattern_noise(coord, y, noise);
  //  
  //  samples.insert(std::make_pair(std::pair<size_t, size_t>(map_x[0], map_x[1]), std::make_pair(y, noise)));
  //}

  void GP_Model::clear_samples() {
    //  gp.clear_sampleset();
    gp.clear();
    samples.clear();
  }

  double GP_Model::f(const size_t map_x[]) {
    Eigen::VectorXd coord(2);
    coord << nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.x(), nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.y();
    //    std::cout << "coord" << std::endl;
    //    std::cout << coord << std::endl;
    return gp.f(coord);
  }

  double GP_Model::var(const size_t map_x[]) {
    Eigen::VectorXd coord(2);
    coord << nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.x(), nc_data.frames[0].cells[map_x[0]][map_x[1]].coord.y();
    return gp.var(coord);
  }

  void GP_Model::plot(const size_t &i, const size_t &j, const double &value) {
    double gray_value = 255.0 - (value - nc_data.min) / nc_data.range * 255.0;
    if (gray_value < 0) {
      gray_value = 0;
    } else if (value > 255.0) {
      gray_value = 255.0;
    }
    image.at<cv::Vec3b>(nc_data.n_rows - i - 1, j) = cv::Vec3b(gray_value, gray_value, gray_value);
  }

  void GP_Model::plot(const size_t &i, const size_t &j, int radius, const cv::Scalar &color, int thickness) {
    cv::circle(image, cv::Point(j, image.rows - i - 1), radius, color, thickness);
  }

  void GP_Model::write(const size_t &n, const size_t &layer, const size_t &point_num) {
    cv::imwrite(dir_name + "/" + file_name + "_" + std::to_string(n) + "_layer" + std::to_string(layer) + "_" + std::to_string(point_num) + "_" + std::to_string(observations.size()) + "obs" + ".png", image);
  }

  void GP_Model::resize_image(const cv::Size &size) {
    resize(image, image, size);
  }

  double GP_Model::Entropy(const Eigen::VectorXi &A) {
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

  double GP_Model::MutualInformation(const Eigen::VectorXi &A, const Eigen::MatrixXi &B) {
    GP_Model temp_gp = *this;
    //    std::cout << "temp_gp.get_samples().size() " << temp_gp.get_samples().size() << std::endl;
    //    std::cout << "this->get_samples().size() " << this->get_samples().size() << std::endl;

    //    std::cout << "B.rows() " << B.rows() << std::endl;
    const clock_t begin_time = clock();
    for (size_t i = 0; i < B.rows(); i++) {
      size_t map_x[] = {static_cast<size_t> (B(i, 0)), static_cast<size_t> (B(i, 1))};
      temp_gp.add_pattern(map_x, 0);
    }
    //    std::cout << "MutualInformation " << static_cast<double> (clock() - begin_time) / CLOCKS_PER_SEC << std::endl;

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
}
