#ifndef GEOMETRY_UTILS_QUATERNION_H
#define GEOMETRY_UTILS_QUATERNION_H

#include <string>
#include <boost/array.hpp>
#include <Eigen/Geometry>
#include <armadillo>
#include "GeometryUtilsMath.h"

namespace geometry_utils
{
  template<typename T>
  struct QuaternionBase : VectorNBase<T, 4>
  {
    QuaternionBase() : VectorNBase<T, 4>()
    {
      this->data.assign(0);
      this->data[0] = 1;
    }

    QuaternionBase(T val) : VectorNBase<T, 4>(val) { }
    QuaternionBase(const QuaternionBase& in) : VectorNBase<T, 4>(in.data) { }
    QuaternionBase(const boost::array<T, 4>& in) : VectorNBase<T, 4>(in) { }
    QuaternionBase(T (&in)[4]) : VectorNBase<T, 4>(in) { }
    QuaternionBase(const arma::vec::fixed<4>& in) : VectorNBase<T, 4>(in) { }
    QuaternionBase(const Eigen::Quaternion<T>& in)
    {
      this->data[0] = in.w();
      this->data[1] = in.x();
      this->data[2] = in.y();
      this->data[3] = in.z();
    }
    QuaternionBase(const VectorNBase<T, 4>& in) : VectorNBase<T, 4>(in) { }

    QuaternionBase(T w, T x, T y, T z)
    {
      this->data[0] = w;
      this->data[1] = x;
      this->data[2] = y;
      this->data[3] = z;
    }

    inline QuaternionBase operator*(const QuaternionBase& rhs) const
    {
      T a1 = this->data[0];
      T b1 = this->data[1];
      T c1 = this->data[2];
      T d1 = this->data[3];
      T a2 = rhs.data[0];
      T b2 = rhs.data[1];
      T c2 = rhs.data[2];
      T d2 = rhs.data[3];
      return QuaternionBase(a1*a2 - b1*b2 - c1*c2 - d1*d2,
                            a2*b1 + a1*b2 - c2*d1 + c1*d2,
                            a2*c1 + a1*c2 + b2*d1 - b1*d2,
                            -(b2*c1) + b1*c2 + a2*d1 + a1*d2);
    }

    inline T& w() { return this->data[0]; }
    inline const T& w() const { return this->data[0]; }

    inline T& x() { return this->data[1]; }
    inline const T& x() const { return this->data[1]; }

    inline T& y() { return this->data[2]; }
    inline const T& y() const { return this->data[2]; }

    inline T& z() { return this->data[3]; }
    inline const T& z() const { return this->data[3]; }

    inline QuaternionBase conj() const
    {
      return QuaternionBase(this->data[0], -this->data[1], -this->data[2], -this->data[3]);
    }

    inline QuaternionBase error(const QuaternionBase& q) const
    {
      return q*(*this).conj();
    }

    inline QuaternionBase axisAngle() const
    {
      QuaternionBase q((*this));
      if (q.w() > 1)
        q = q/q.norm();
      T den = math::sqrt(1-q.w()*q.w());
      if (den < 1e-6)
        den = 1;
      return QuaternionBase(2*math::acos(q.w()), q.x()/den, q.y()/den, q.z()/den);
    }
  };

  typedef QuaternionBase<float> Quaternionf;
  typedef QuaternionBase<float> Quatf;

  typedef QuaternionBase<double> Quaterniond;
  typedef QuaternionBase<double> Quatd;

  typedef QuaternionBase<double> Quaternion;
  typedef QuaternionBase<double> Quat;
}
#endif
