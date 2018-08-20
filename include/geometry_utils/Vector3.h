#ifndef GEOMETRY_UTILS_VECTOR3_H
#define GEOMETRY_UTILS_VECTOR3_H

#include "VectorNBase.h"

namespace geometry_utils
{
  template<typename T>
  struct Vector3Base : VectorNBase<T, 3>
  {
    Vector3Base() : VectorNBase<T, 3>() { }
    Vector3Base(T val) : VectorNBase<T, 3>(val) { }
    Vector3Base(const Vector3Base& in) : VectorNBase<T, 3>(in.data) { }
    Vector3Base(const boost::array<T, 3>& in) : VectorNBase<T, 3>(in) { }
    Vector3Base(T (&in)[3]) : VectorNBase<T, 3>(in) { }
    Vector3Base(const arma::vec::fixed<3>& in) : VectorNBase<T, 3>(in) { }
    Vector3Base(const Eigen::Matrix<T, 3, 1>& in) : VectorNBase<T, 3>(in) { }
    Vector3Base(const VectorNBase<T, 3>& in) : VectorNBase<T, 3>(in) { }


    using VectorNBase<T, 3>::data; 
	//or use this-> before data[] to break templated hidden inheritance

    Vector3Base(T v1, T v2, T v3)
    {
      data[0] = v1;
      data[1] = v2;
      data[2] = v3;
    }

    T& x() { return data[0]; }
    T& y() { return data[1]; }
    T& z() { return data[2]; }
    T x() const { return data[0]; }
    T y() const { return data[1]; }
    T z() const { return data[2]; }

    T& r() { return data[0]; }
    T& g() { return data[1]; }
    T& b() { return data[2]; }
    T& r() const { return data[0]; }
    T& g() const { return data[1]; }
    T& b() const { return data[2]; }

    void setx(T x) { data[0]=x; }
    void sety(T y) { data[1]=y; }
    void setz(T z) { data[2]=z; }
    void set(T d0, T d1, T d2) 
    {
      data[0] = d0;
      data[1] = d1;
      data[2] = d2; 
    }

    void scale(double d0, double d1, double d2) 
    {
      data[0] *= d0;
      data[1] *= d1;
      data[2] *= d2; 
    }

    void scale(double d) { 
      scale(d,d,d); 
    }
    void negate() { 
      scale(-1.0); 
    }

    inline Vector3Base<T> cross(const Vector3Base<T>& v) const
    {
      return Vector3Base<T>(-(*this)(2)*v(1) + (*this)(1)*v(2),
                            (*this)(2)*v(0) - (*this)(0)*v(2),
                            -(*this)(1)*v(0) + (*this)(0)*v(1));
    }
  };


  inline Vector3Base<float> operator*(const float& lhs, const Vector3Base<float>& rhs)
  {
    return Vector3Base<float>(rhs*lhs);
  }

  inline Vector3Base<double> operator*(const double& lhs, const Vector3Base<double>& rhs)
  {
    return Vector3Base<double>(rhs*lhs);
  }

  template<typename T>
  inline VectorNBase<T, 3> cross(const VectorNBase<T, 3>& v1,
                                 const VectorNBase<T, 3>& v2)
  {
    return Vector3Base<T>(v1).cross(v2);
  }

  typedef Vector3Base<float> Vector3f;
  typedef Vector3Base<float> Vec3f;

  typedef Vector3Base<double> Vector3d;
  typedef Vector3Base<double> Vec3d;

  typedef Vector3Base<double> Vector3;
  typedef Vector3Base<double> Vec3;
}
#endif
