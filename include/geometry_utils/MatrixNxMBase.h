#ifndef GEOMETRY_UTILS_MATRIXNXM_H
#define GEOMETRY_UTILS_MATRIXNXM_H

#include <ostream>
#include "VectorNBase.h"
#include "GeometryUtilsMath.h"

namespace geometry_utils
{
  template<typename T, size_t N, size_t M>
  struct MatrixNxMBase
  {
    typedef typename boost::shared_ptr< MatrixNxMBase<T, N, M> > Ptr;
    typedef typename boost::shared_ptr< const MatrixNxMBase<T, N, M> > ConstPtr;

    static const size_t size = N*M;
    static const size_t nrows = N;
    static const size_t ncols = M;

    boost::array<T, size> data;

    MatrixNxMBase() { data.fill(0); }

    MatrixNxMBase(T val) { data.fill(val); }

    MatrixNxMBase(const MatrixNxMBase& in) : data(in.data) { }

    MatrixNxMBase(const boost::array<T, size>& in) : data(in) { }

    MatrixNxMBase(T (&in)[size])
    {
      for (unsigned int i = 0; i < size; i++)
        data[i] = in[i];
    }

    MatrixNxMBase(const arma::mat::fixed<N, M>& in)
    {
      for (size_t i = 0; i < nrows; i++)
        for (size_t j = 0; j < ncols; j++)
          data[ncols*i + j] = in(i, j);
    }

    MatrixNxMBase(const Eigen::Matrix<T, N, M>& in)
    {
      for (size_t i = 0; i < nrows; i++)
        for (size_t j = 0; j < ncols; j++)
          data[ncols*i + j] = in(i, j);
    }

    inline T& operator()(unsigned int i) { return data[i]; }
    inline const T& operator()(unsigned int i) const { return data[i]; }

    inline T& get(unsigned int i) { return data[i]; }
    inline const T& get(unsigned int i) const { return data[i]; }

    inline T& operator()(unsigned int i, unsigned int j)
    {
      return data[ncols*i + j];
    }

    inline const T& operator()(unsigned int i, unsigned int j) const
    {
      return data[ncols*i + j];
    }

    inline T& get(unsigned int i, unsigned int j)
    {
      return data[ncols*i + j];
    }

    inline const T& get(unsigned int i, unsigned int j) const
    {
      return data[ncols*i + j];
    }

    inline std::ostream& operator<<(std::ostream& out)
    {
      out << data;
      return out;
    }

    inline MatrixNxMBase& operator=(const MatrixNxMBase& rhs)
    {
      if (this == &rhs)
        return *this;
      data = rhs.data;
      return *this;
    }

    inline MatrixNxMBase<T, N, M> operator*(T rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i]*rhs;
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator+(const MatrixNxMBase& rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i] + rhs(i);
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator-() const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = -data[i];
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator-(const MatrixNxMBase& rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i] - rhs(i);
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, N> operator*(const MatrixNxMBase<T, M, N>& rhs) const
    {
      T d[N*N];
      for (size_t i = 0; i < N; i++)
        for (size_t j = 0; j < N; j++)
          d[N*i + j] = this->row(i)^rhs.col(j);
      return MatrixNxMBase<T, N, N>(d);
    }

    inline VectorNBase<T, N> operator*(const VectorNBase<T, M>& rhs) const
    {
      T d[nrows];
      for (size_t i = 0; i < nrows; i++)
        d[i] = this->row(i)^rhs;
      return VectorNBase<T, N>(d);
    }

    inline MatrixNxMBase<T, N, M> operator%(const MatrixNxMBase& rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i]*rhs(i);
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator/(const MatrixNxMBase& rhs) const
    {
      T d[size];
      for (size_t i = 0; i < size; i++)
        d[i] = data[i]/rhs(i);
      return MatrixNxMBase<T, N, M>(d);
    }

    inline MatrixNxMBase<T, N, M> operator+=(const MatrixNxMBase& rhs)
    {
      for (size_t i = 0; i < size; i++)
        data[i] += rhs.data[i];
      return *this;
    }

    inline MatrixNxMBase<T, N, M> operator-=(const MatrixNxMBase& rhs)
    {
      for (size_t i = 0; i < size; i++)
        data[i] -= rhs.data[i];
      return *this;
    }

    inline MatrixNxMBase<T, N, M> operator%=(const MatrixNxMBase& rhs)
    {
      for (size_t i = 0; i < size; i++)
        data[i] *= rhs.data[i];
      return *this;
    }

    inline MatrixNxMBase<T, N, M> operator/=(const MatrixNxMBase& rhs)
    {
      for (size_t i = 0; i < size; i++)
        data[i] /= rhs.data[i];
      return *this;
    }

    inline MatrixNxMBase<T, M, N> trans() const
    {
      T d[size];
      for (size_t i = 0; i < nrows; i++)
        for (size_t j = 0; j < ncols; j++)
          d[nrows*j + i] = data[ncols*i + j];
      return MatrixNxMBase<T, M, N>(d);
    }

    inline MatrixNxMBase<T, M, N> t() const
    {
      return this->trans();
    }

    inline VectorNBase<T, M> row(unsigned int r) const
    {
      T d[ncols];
      for (size_t i = 0; i < ncols; i++)
        d[i] = data[ncols*r + i];
      return VectorNBase<T, M>(d);
    }

    inline VectorNBase<T, N> col(unsigned int c) const
    {
      T d[nrows];
      for (size_t i = 0; i < nrows; i++)
        d[i] = data[ncols*i + c];
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

    inline MatrixNxMBase<T, N, M> scale(T s) const
    {
      return (*this)*s;
    }

    inline void print(const std::string& prefix = std::string()) const
    {
      if (!prefix.empty())
        std::cout << prefix << std::endl;
      std::cout << (*this) << std::endl;
    }

    inline arma::mat::fixed<N, M> arma() const
    {
      return arma::trans(arma::mat::fixed<M, N>(data.data()));
    }

    inline Eigen::Matrix<T, N, N> eigen() const
    {
      return Eigen::Matrix<T, N, N>(data.data()).transpose();
    }

    inline bool operator==(const MatrixNxMBase& that) const
    {
      return this->equals(that);
    }

    inline bool operator!=(const MatrixNxMBase& that) const
    {
      return !this->equals(that);
    }

    virtual inline bool equals(const MatrixNxMBase& that, const T ptol = 1e-8) const
    {
      return (*this - that).norm() < ptol;
    }

    inline T norm() const
    {
      return math::sqrt((this->trans()*(*this)).trace());
    }

    inline T trace()
    {
      size_t count = nrows <= ncols ? nrows : ncols;
      T tr = 0;
      for (size_t i = 0; i < count; i++)
        tr += data[ncols*i + i];
      return tr;
    }
  };

  template<size_t N, size_t M>
  inline MatrixNxMBase<float, N, M> operator*(const float& lhs,
                                              const MatrixNxMBase<float, N, M>& rhs)
  {
    return rhs*lhs;
  }

  template<size_t N, size_t M>
  inline MatrixNxMBase<double, N, M> operator*(const double& lhs,
                                               const MatrixNxMBase<double, N, M>& rhs)
  {
    return rhs*lhs;
  }

  template<typename T, size_t N, size_t M>
  inline std::ostream& operator<<(std::ostream& out, const MatrixNxMBase<T, N, M>& m)
  {
    for (size_t i = 0; i < N; i++)
    {
      for (size_t j = 0; j < M; j++)
        out << m.data[M*i + j] << " ";
      out << std::endl;
    }
    return out;
  }

  template<typename T, size_t N, size_t M>
  inline arma::mat::fixed<N, M> arma(const MatrixNxMBase<T, N, M>& in)
  {
    return in.arma();
  }

  template<typename T, size_t N, size_t M>
  inline Eigen::Matrix<T, N, M> eigen(const MatrixNxMBase<T, N, M>& in)
  {
    return in.eigen();
  }

  template<typename T, size_t N, size_t M>
  inline MatrixNxMBase<T, M, N> trans(const MatrixNxMBase<T, N, M>& m)
  {
    return m.trans();
  }

  template<typename T, size_t N, size_t M>
  inline MatrixNxMBase<T, N, M> outer(const VectorNBase<T, N>& v1,
                                      const VectorNBase<T, M>& v2)
  {
    T d[N*M];
    for (size_t i = 0; i < N; i++)
      for (size_t j = 0; j < M; j++)
        d[M*i + j] = v1(i)*v2(j);
    return MatrixNxMBase<T, N, M>(d);
  }
}
#endif
