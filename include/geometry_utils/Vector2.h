#ifndef GEOMETRY_UTILS_VECTOR2_H
#define GEOMETRY_UTILS_VECTOR2_H

#include "VectorNBase.h"

namespace geometry_utils
{
  template<typename T>
  struct Vector2Base : VectorNBase<T, 2>
  {
    Vector2Base() : VectorNBase<T, 2>() { }
    Vector2Base(T val) : VectorNBase<T, 2>(val) { }
    Vector2Base(const Vector2Base& in) : VectorNBase<T, 2>(in.data) { }
    Vector2Base(const boost::array<T, 2>& in) : VectorNBase<T, 2>(in) { }
    Vector2Base(T (&in)[2]) : VectorNBase<T, 2>(in) { }
    Vector2Base(const arma::vec::fixed<2>& in) : VectorNBase<T, 2>(in) { }
    Vector2Base(const Eigen::Matrix<T, 2, 1>& in) : VectorNBase<T, 2>(in) { }
    Vector2Base(const VectorNBase<T, 2>& in) : VectorNBase<T, 2>(in) { }

    using VectorNBase<T, 2>::data;
        //or use  before data[] to break templated hidden inheritance

    Vector2Base(T v1, T v2)
    {
      data[0] = v1;
      data[1] = v2;
    }

    T& x() { return data[0]; }
    T& y() { return data[1]; }
    T x() const { return data[0]; }
    T y() const { return data[1]; }

    void setx(T x) { data[0]=x; }
    void sety(T y) { data[1]=y; }
    void set(T d0, T d1)
    {
      data[0] = d0;
      data[1] = d1;
    }

    void scale(double d0, double d1)
    {
      data[0] *= d0;
      data[1] *= d1;
    }

    void scale(double d) {
      scale(d,d);
    }
    void negate() {
      scale(-1.0);
    }

  };

  template<typename T>
  inline Vector2Base<T> operator*(const T& lhs, const Vector2Base<T>& rhs) {
    return Vector2Base<T>(rhs*lhs);
  }

  typedef Vector2Base<int> Vector2i;
  typedef Vector2Base<int> Vec2i;
  
  typedef Vector2Base<float> Vector2f;
  typedef Vector2Base<float> Vec2f;

  typedef Vector2Base<double> Vector2d;
  typedef Vector2Base<double> Vec2d;

  typedef Vector2Base<double> Vector2;
  typedef Vector2Base<double> Vec2;
}
#endif
