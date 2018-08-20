#ifndef GEOMETRY_UTILS_VECTORN_H
#define GEOMETRY_UTILS_VECTORN_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <armadillo>
#include <Eigen/Core>
#include "GeometryUtilsMath.h"

namespace geometry_utils
{
  template<typename T, size_t N>
  struct VectorNBase
  {
    typedef typename boost::shared_ptr< VectorNBase<T, N> > Ptr;
    typedef typename boost::shared_ptr< const VectorNBase<T, N> > ConstPtr;

    static const size_t length = N;

    boost::array<T, N> data;

    VectorNBase() { data.fill(0); }

    VectorNBase(T val) { data.fill(val); }

    VectorNBase(const VectorNBase& in) : data(in.data) { }

    VectorNBase(const boost::array<T, N>& in) : data(in) { }

    VectorNBase(T (&in)[N])
    {
      for (size_t i = 0; i < N; i++)
        data[i] = in[i];
    }

    VectorNBase(const arma::vec::fixed<N>& in)
    {
      for (size_t i = 0; i < N; i++)
        data[i] = in(i);
    }

    VectorNBase(const Eigen::Matrix<T, N, 1>& in)
    {
      for (size_t i = 0; i < N; i++)
        data[i] = in(i, 0);
    }

    inline T& operator()(unsigned int i) { return data[i]; }
    inline T& operator[](unsigned int i) { return data[i]; }

    inline const T& operator()(unsigned int i) const { return data[i]; }

    inline T& get(unsigned int i) { return data[i]; }
    inline const T& get(unsigned int i) const { return data[i]; }

    inline VectorNBase& operator=(const VectorNBase& rhs)
    {
      if (this == &rhs)
        return *this;
      data = rhs.data;
      return *this;
    }

    inline VectorNBase operator*(T rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i]*rhs;
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator+(const VectorNBase& rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i] + rhs.data[i];
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator-() const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = -data[i];
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator-(const VectorNBase& rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i] - rhs.data[i];
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator%(const VectorNBase& rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i]*rhs.data[i];
      return VectorNBase<T, N>(d);
    }

    inline T operator^(const VectorNBase& rhs) const
    {
      T dot = 0;
      for (size_t i = 0; i < N; i++)
        dot += data[i]*rhs.data[i];
      return dot;
    }

    inline VectorNBase operator/(const VectorNBase& rhs) const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = data[i]/rhs.data[i];
      return VectorNBase<T, N>(d);
    }

    inline VectorNBase operator+=(const VectorNBase& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] += rhs.data[i];
      return *this;
    }

    inline VectorNBase operator-=(const VectorNBase& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] -= rhs.data[i];
      return *this;
    }

    inline VectorNBase operator%=(const VectorNBase& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] *= rhs.data[i];
      return *this;
    }

    inline VectorNBase operator/=(const VectorNBase& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] /= rhs.data[i];
      return *this;
    }

    inline VectorNBase operator*=(const T& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] *= rhs;
      return *this;
    }

    inline VectorNBase operator/=(const T& rhs)
    {
      for (size_t i = 0; i < N; i++)
        data[i] /= rhs;
      return *this;
    }
    
    inline bool operator==(const VectorNBase& that) const
    {
      return this->equals(that);
    }

    inline bool operator!=(const VectorNBase& that) const
    {
      return !this->equals(that);
    }

    inline bool equals(const VectorNBase& that, const T ptol = 1e-8) const
    {
      return (*this - that).norm() <= ptol;
    }

    inline T norm() const
    {
      return math::sqrt((*this)^(*this));
    }

    inline T sum() const
    {
      T sum = 0;
      for (size_t i = 0; i < N; i++)
        sum += data[i];
      return sum;
    }

/*
    inline VectorNBase normalize() const
    {
      return (*this)/norm();
    }
*/

    inline void normalize()
    {
      VectorNBase v = (*this)/norm();
      this->data =  v.data;
    }

    inline VectorNBase abs() const
    {
      T d[N];
      for (size_t i = 0; i < N; i++)
        d[i] = std::abs(data[i]);
      return VectorNBase<T, N>(d);
    }

    inline void ones()
    {
      data.fill(1);
    }

    inline void zeros()
    {
      data.fill(0);
    }

    inline T dot(const VectorNBase& v) const
    {
      return (*this)^v;
    }

    inline VectorNBase scale(T s) const
    {
      return (*this)*s;
    }

    inline void print(const std::string& prefix = std::string()) const
    {
      if (!prefix.empty())
        std::cout << prefix << std::endl;
      std::cout << (*this) << std::endl;
    }

    inline arma::vec::fixed<N> arma() const
    {
      return arma::vec::fixed<N>(data.data());
    }

    inline Eigen::Matrix<T, N, 1> eigen() const
    {
      return Eigen::Matrix<T, N, 1>(data.data());
    }
  };

  template<typename T, size_t N>
  inline VectorNBase<T, N> operator*(const T& lhs, const VectorNBase<T, N>& rhs)
  {
    return rhs*lhs;
  }

  template<typename T, size_t N>
  inline std::ostream& operator<<(std::ostream& out, const VectorNBase<T, N>& m)
  {
    for (size_t i = 0; i < N - 1; i++)
      out << m.data[i] << " ";
    out << m.data[N-1];
    return out;
  }

  template<typename T, size_t N>
  inline arma::vec::fixed<N> arma(const VectorNBase<T, N>& in)
  {
    return in.arma();
  }

  template<typename T, size_t N>
  inline Eigen::Matrix<T, N, 1> eigen(const VectorNBase<T, N>& in)
  {
    return in.eigen();
  }

  template<typename T, size_t N>
  inline T norm(const VectorNBase<T, N>& v)
  {
    return v.norm();
  }

  template<typename T, size_t N>
  inline T dot(const VectorNBase<T, N>& v1, const VectorNBase<T, N>& v2)
  {
    return v1.dot(v2);
  }
}
#endif
