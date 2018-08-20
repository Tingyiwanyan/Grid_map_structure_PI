
#ifndef _UTILS_FUNCTIONS_H
#define _UTILS_FUNCTIONS_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <limits>
#include <assert.h>
#include <iostream>
#include <vector>

namespace utils {


  /////////////////////////////// Math /////////////////////////////

  inline double gaussian_pdf(double x, double m, double s) {
    static const double inv_sqrt_2pi = 0.3989422804014327;
    double a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-0.5 * a * a);
  }


  //box-muller transform to generate 1D gaussian (wiki noted it is faster)

  inline double generateGaussianNoise(double mu, double sigma) {
    static const double epsilon = std::numeric_limits<double>::min();
    //const double two_pi = 2.0*3.14159265358979323846;
    static const double two_pi = 2.0 * M_PI;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
      return z1 * sigma + mu;

    double u1, u2;
    do {
      u1 = rand() * (1.0 / RAND_MAX);
      u2 = rand() * (1.0 / RAND_MAX);
    } while (u1 <= epsilon);

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
  }


  //CDF of standard normal distribution, see: http://www.johndcook.com/blog/cpp_phi/

  inline double phiStandardNormal(double x) {
    // constants
    double a1 = 0.254829592;
    double a2 = -0.284496736;
    double a3 = 1.421413741;
    double a4 = -1.453152027;
    double a5 = 1.061405429;
    double p = 0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
      sign = -1;
    x = fabs(x) / sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0 / (1.0 + p * x);
    double y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * exp(-x * x);

    return 0.5 * (1.0 + sign * y);
  }

  inline double pdfBivariateGaussian(double x, double y, double mx, double my,
          double sigma_x, double sigma_y, double sigma_xy) {
    double rho = sigma_xy / (sigma_x * sigma_y);
    assert(fabs(rho) <= 1.0);
    double coeff = 1.0 / (2 * M_PI * sigma_x * sigma_y * sqrt(1 - rho * rho));
    double sigma_poly = (x - mx)*(x - mx) / (sigma_x * sigma_x) - 2 * rho * (x - mx)*(y - my) / (sigma_x * sigma_y) + (y - my)*(y - my) / (sigma_y * sigma_y);

    return coeff * exp(-0.5 / (1 - rho * rho) * sigma_poly);

  }

  template<typename T>
  std::vector<T> resampling1D(const std::vector<T> &v, const double &scaling) {
    const unsigned int resampling_size = v.size() * scaling;
    std::vector<T> resampled_vector(resampling_size);

    for (unsigned int i = 0; i < resampling_size; i++) {
      const unsigned int x1 = std::floor(i / scaling);
      const unsigned int x2 = std::ceil(i / scaling);
      const T value1 = v[std::min(x1, static_cast<unsigned int> (v.size() - 1))];
      const T value2 = v[std::min(x2, static_cast<unsigned int> (v.size() - 1))];

      const double alpha = i / scaling - std::floor(i / scaling);
      const T value = value1 + alpha * (value2 - value1);

      resampled_vector[i] = value;
    }
    return resampled_vector;
  }

  template<typename T>
  std::vector<std::vector<T>> resampling2D(const std::vector<std::vector<T>> &v, const unsigned int &size_x, const unsigned int &size_y, const double &scaling_x, const double &scaling_y) {
    const unsigned int scaled_x_size = size_x * scaling_x;
    const unsigned int scaled_y_size = size_y * scaling_y;
    
    std::vector<std::vector<T>> resampled_vector(scaled_y_size);
    for (auto& c : resampled_vector) {
      c.resize(scaled_x_size);
    }

    for (unsigned int i = 0; i < scaled_y_size; i++) {
      for (unsigned int j = 0; j < scaled_x_size; j++) {

        const unsigned int x1 = std::min(static_cast<unsigned int>(std::floor(j / scaling_x)), size_x - 1);
        const unsigned int y1 = std::min(static_cast<unsigned int>(std::floor(i / scaling_y)), size_y - 1);
        
        const unsigned int x2 = std::min(static_cast<unsigned int>(std::ceil(j / scaling_x)), size_x - 1);
        const unsigned int y2 = std::min(static_cast<unsigned int>(std::floor(i / scaling_y)), size_y - 1);

        const unsigned int x3 = std::min(static_cast<unsigned int>(std::floor(j / scaling_x)), size_x - 1);
        const unsigned int y3 = std::min(static_cast<unsigned int>(std::ceil(i / scaling_y)), size_y - 1);

        const unsigned int x4 = std::min(static_cast<unsigned int>(std::ceil(j / scaling_x)), size_x - 1);
        const unsigned int y4 = std::min(static_cast<unsigned int>(std::ceil(i / scaling_y)), size_y - 1);

        const T value1 = v[y1][x1];
        const T value2 = v[y2][x2];
        const T value3 = v[y3][x3];
        const T value4 = v[y4][x4];

        const double alpha = j / scaling_x - std::floor(j / scaling_x);
        const double beta = i / scaling_y - std::floor(i / scaling_y);
        const T value = value1 + alpha * (value2 - value1) + beta * (value3 - value1) + alpha * beta * (value1 - value2 - value3 + value4);

        resampled_vector[i][j] = value;
      }
    }
    return resampled_vector;
  }
  
    template<typename T>
  std::vector<T> resampling2D(const std::vector<T> &v, const unsigned int &size_x, const unsigned int &size_y, const double &scaling_x, const double &scaling_y) {
    const unsigned int scaled_x_size = size_x * scaling_x;
    const unsigned int scaled_y_size = size_y * scaling_y;
    std::vector<T> resampled_vector(scaled_x_size * scaled_y_size);

    for (unsigned int i = 0; i < scaled_y_size; i++) {
      for (unsigned int j = 0; j < scaled_x_size; j++) {

        const unsigned int x1 = std::min(static_cast<unsigned int>(std::floor(j / scaling_x)), size_x - 1);
        const unsigned int y1 = std::min(static_cast<unsigned int>(std::floor(i / scaling_y)), size_y - 1);
        
        const unsigned int x2 = std::min(static_cast<unsigned int>(std::ceil(j / scaling_x)), size_x - 1);
        const unsigned int y2 = std::min(static_cast<unsigned int>(std::floor(i / scaling_y)), size_y - 1);

        const unsigned int x3 = std::min(static_cast<unsigned int>(std::floor(j / scaling_x)), size_x - 1);
        const unsigned int y3 = std::min(static_cast<unsigned int>(std::ceil(i / scaling_y)), size_y - 1);

        const unsigned int x4 = std::min(static_cast<unsigned int>(std::ceil(j / scaling_x)), size_x - 1);
        const unsigned int y4 = std::min(static_cast<unsigned int>(std::ceil(i / scaling_y)), size_y - 1);

        const T value1 = v[y1 * size_x + x1];
        const T value2 = v[y2 * size_x + x2];
        const T value3 = v[y3 * size_x + x3];
        const T value4 = v[y4 * size_x + x4];

        const double alpha = j / scaling_x - std::floor(j / scaling_x);
        const double beta = i / scaling_y - std::floor(i / scaling_y);
        const T value = value1 + alpha * (value2 - value1) + beta * (value3 - value1) + alpha * beta * (value1 - value2 - value3 + value4);

        resampled_vector[i * scaled_x_size + j] = value;
      }
    }
    return resampled_vector;
  }
}


///////////////////////////// Utils /////////////////////////////


//overload std::cout for printing vector<T>

template <typename T >
inline std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
  os << "[";
  std::copy(v.begin(), v.end(), std::ostream_iterator<T>(std::cout, " "));
  os << " ]";
  return os;
}


#endif
