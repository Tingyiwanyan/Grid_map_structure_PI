#ifndef GEOMETRY_UTILS_VECTOR4_H
#define GEOMETRY_UTILS_VECTOR4_H

#include "VectorNBase.h"

namespace geometry_utils
{
  template<typename T>
  struct Vector4Base : VectorNBase<T, 4>
  {
    Vector4Base() : VectorNBase<T, 4>() { }
    Vector4Base(T val) : VectorNBase<T, 4>(val) { }
    Vector4Base(const Vector4Base& in) : VectorNBase<T, 4>(in.data) { }
    Vector4Base(const boost::array<T, 4>& in) : VectorNBase<T, 4>(in) { }
    Vector4Base(T (&in)[4]) : VectorNBase<T, 4>(in) { }
    Vector4Base(const arma::vec::fixed<4>& in) : VectorNBase<T, 4>(in) { }
    Vector4Base(const Eigen::Matrix<T, 4, 1>& in) : VectorNBase<T, 4>(in) { }
    Vector4Base(const VectorNBase<T, 4>& in) : VectorNBase<T, 4>(in) { }

    using VectorNBase<T, 4>::data;
        //or use this-> before data[] to break templated hidden inheritance

    Vector4Base(T v1, T v2, T v3, T v4)
    {
      data[0] = v1;
      data[1] = v2;
      data[2] = v3;
      data[3] = v4;
    }

/*
    //defined in Quaternion.h
    T& x() { return data[0]; }
    T& y() { return data[1]; }
    T& z() { return data[2]; }
    T& w() { return data[3]; }
    T x() const { return data[0]; }
    T y() const { return data[1]; }
    T z() const { return data[2]; }
    T w() const { return data[3]; }
*/

    T& r() { return data[0]; }
    T& g() { return data[1]; }
    T& b() { return data[2]; }
    T& a() { return data[3]; }
    T r() const { return data[0]; }
    T g() const { return data[1]; }
    T b() const { return data[2]; }
    T a() const { return data[3]; }

    void set(T d0, T d1, T d2, T d3)
    {
      data[0] = d0;
      data[1] = d1;
      data[2] = d2;
      data[3] = d3;
    }

    void scale(double d0, double d1, double d2, double d3)
    {
      data[0] *= d0;
      data[1] *= d1;
      data[2] *= d2;
      data[3] *= d3;
    }

    void scale(double d) {
      scale(d,d,d);
    }
    void negate() {
      scale(-1.0);
    }

  };


  inline Vector4Base<float> operator*(const float& lhs, const Vector4Base<float>& rhs)
  {
    return Vector4Base<float>(rhs*lhs);
  }

  inline Vector4Base<double> operator*(const double& lhs, const Vector4Base<double>& rhs)
  {
    return Vector4Base<double>(rhs*lhs);
  }

  typedef Vector4Base<float> Vector4f;
  typedef Vector4Base<float> Vec4f;

  typedef Vector4Base<double> Vector4d;
  typedef Vector4Base<double> Vec4d;

  typedef Vector4Base<double> Vector4;
  typedef Vector4Base<double> Vec4;
}
#endif
