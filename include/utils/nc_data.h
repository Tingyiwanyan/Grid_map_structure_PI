
#ifndef NC_DATA_H
#define NC_DATA_H

#include <cstddef>
#include <gp.h>
#include "geometry_utils/Pose.h"
#include "functions.h"

using namespace std;
namespace gu = geometry_utils;

namespace utils {

  typedef struct Cell {
    gu::Vec2 coord;
    double temperature;
    double salinity;
    gu::Vec2 ocean_vec;

    Cell operator+(const Cell &rhs) const {
      Cell new_c;
      new_c.coord = coord + rhs.coord;
      new_c.temperature = temperature + rhs.temperature;
      new_c.salinity = salinity + rhs.salinity;
      new_c.ocean_vec = ocean_vec + rhs.ocean_vec;
      return new_c;
    }

    Cell operator-(const Cell &rhs) const {
      Cell new_c;
      new_c.coord = coord - rhs.coord;
      new_c.temperature = temperature - rhs.temperature;
      new_c.salinity = salinity - rhs.salinity;
      new_c.ocean_vec = ocean_vec - rhs.ocean_vec;
      return new_c;
    }

    template<typename T>
    friend Cell operator*(T lhs, const Cell &rhs) {
      Cell new_c;
      new_c.coord.set(lhs * rhs.coord.x(), lhs * rhs.coord.y());
      new_c.temperature = lhs * rhs.temperature;
      new_c.salinity = lhs * rhs.salinity;
      new_c.ocean_vec.set(lhs * rhs.ocean_vec.x(), lhs * rhs.ocean_vec.y());
      return new_c;
    }

  } cell_t;

  typedef struct Frame {
    double time;
    vector<vector<cell_t>> cells;

    Frame(const unsigned int &n_rows, const unsigned int &n_cols) {
      cells.resize(n_rows);
      for (auto& c : cells) {
        c.resize(n_cols);
      }
    }

    ~Frame() {
    }

  } frame_t;

  typedef struct NC_Data {
    size_t n_rows, n_cols;
    vector<frame_t> frames;
    double max;
    double min;
    double range;
  } nc_data_t;

  inline nc_data_t resamplingNC(const nc_data_t &nc, const double &scaling_x, const double &scaling_y) {
    nc_data_t new_nc_data;
    new_nc_data.n_rows = nc.n_rows * scaling_y;
    new_nc_data.n_cols = nc.n_cols * scaling_x;

    for (int i = 0; i < nc.frames.size(); i++) {
      frame_t new_f(new_nc_data.n_rows, new_nc_data.n_cols);
      new_f.time = nc.frames[i].time;
      new_f.cells = utils::resampling2D(nc.frames[i].cells, nc.n_cols, nc.n_rows, scaling_x, scaling_y);
      new_nc_data.frames.push_back(new_f);
    }

    return new_nc_data;
  }
  
  inline nc_data_t extractSubRegionNC(const nc_data_t &full_resolution_nc, const unsigned int &origin_lat, const unsigned int &origin_lon, const unsigned int &top_right_lat, const unsigned int &top_right_lon) {
    nc_data_t new_nc_data;
    new_nc_data.n_rows = top_right_lat - origin_lat;
    new_nc_data.n_cols = top_right_lon - origin_lon;
    
    std::cout << "full_resolution_nc full_resolution_nc.n_rows " << full_resolution_nc.n_rows << std::endl;
    std::cout << "full_resolution_nc full_resolution_nc.n_cols " << full_resolution_nc.n_cols << std::endl;

    for (int i = 0; i < full_resolution_nc.frames.size(); i++) {
      frame_t new_frame(new_nc_data.n_rows, new_nc_data.n_cols);
      new_frame.time = full_resolution_nc.frames[i].time;
      for (int j = 0; j < new_nc_data.n_rows; j++) {
          for (int k = 0; k < new_nc_data.n_cols; k++) {
              cell_t c;
              c.coord = full_resolution_nc.frames[i].cells[origin_lat + j][origin_lon + k].coord;
              c.ocean_vec = full_resolution_nc.frames[i].cells[origin_lat + j][origin_lon + k].ocean_vec;
              c.salinity = full_resolution_nc.frames[i].cells[origin_lat + j][origin_lon + k].salinity;
              c.temperature = full_resolution_nc.frames[i].cells[origin_lat + j][origin_lon + k].temperature;
              new_frame.cells[j][k] = c;
          }
      }
      new_nc_data.frames.push_back(new_frame);
    }

    return new_nc_data;
  }

  inline std::vector<libgp::GaussianProcess *> generateGPs(const nc_data_t &nc, const unsigned int &input_dim, const std::string &cov_func, const std::vector<double> &cov_params, const bool &u) {
    std::vector<libgp::GaussianProcess *> gps(nc.n_rows * nc.n_cols, nullptr);
    for (int i = 0; i < gps.size(); i++) {
      // initialize Gaussian process for 1-D input using the squared exponential covariance function
      gps[i] = new libgp::GaussianProcess(input_dim, cov_func);

      // set parameters of covariance function
      if (gps[i]->covf().get_param_dim() != cov_params.size()) {
        std::cerr << "wrong parameters" << std::endl;
      }
      Eigen::VectorXd params(gps[i]->covf().get_param_dim());
      params << cov_params[0], cov_params[1];
      gps[i]->covf().set_loghyper(params);
    }

    for (int i = 0; i < nc.frames.size(); i++) {
      for (int j = 0; j < nc.n_rows; j++) {
        for (int k = 0; k < nc.n_cols; k++) {
          double x = i; // nc.frames[i].time;
          double y = NAN;

          if (u) {
            y = nc.frames[i].cells[j][k].ocean_vec.x();
          } else {
            y = nc.frames[i].cells[j][k].ocean_vec.y();
          }

          gps[j * nc.n_cols + k]->add_pattern(&x, y);
        }
      }
    }
    return gps;
  }

  inline void deleteGPs(std::vector<libgp::GaussianProcess *> gps) {
    for (auto gp : gps) {
      delete gp;
    }
  }

}


#endif


