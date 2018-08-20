#ifndef GEOMETRY_UTILS_TRANSFORM2_H
#define GEOMETRY_UTILS_TRANSFORM2_H

#include "Vector2.h"
#include "Rotation2.h"

namespace geometry_utils
{
  template<typename T>
  struct Transform2Base
  {
    typedef boost::shared_ptr<Transform2Base> Ptr;
    typedef boost::shared_ptr<const Transform2Base> ConstPtr;

    Vector2Base<T> translation;
    Rotation2Base<T> rotation;

    Transform2Base()
    {
      translation.zeros();
      rotation.eye();
    }

    Transform2Base(const Vector2Base<T>& translation_,
                   const Rotation2Base<T>& rotation_) :
      translation(translation_), rotation(rotation_) { }

    Transform2Base(const Transform2Base<T>& in) :
      translation(in.translation), rotation(in.rotation) { }

    Transform2Base(T x, T y, T th) : translation(x, y), rotation(th) {}

    Transform2Base(const Vector2Base<T>& translation_) :
      translation(translation_) { rotation.eye(); }

    Transform2Base& operator=(const Transform2Base& rhs)
    {
      if (this == &rhs)
        return *this;
      translation = rhs.translation;
      rotation = rhs.rotation;
      return *this;
    }

    Vector2Base<T> operator*(const Vector2Base<T>& p) const
    {
      return rotation*p + translation;
    }

    Transform2Base<T> operator+(const Transform2Base<T>& t) const
    {
      return Transform2Base<T>(translation + rotation*t.translation,
                               rotation*t.rotation);
    }

    bool operator==(const Transform2Base& that) const
    {
      return this->equals(that);
    }

    bool operator!=(const Transform2Base& that) const
    {
      return !this->equals(that);
    }

    bool equals(const Transform2Base& that,
                const T ptol = 1e-5,
                const T rtol = 1e-5) const
    {
      return (translation.equals(that.translation, ptol) &&
              rotation.equals(that.rotation, rtol));
    }

    void print(const std::string& prefix = std::string()) const
    {
      if (!prefix.empty())
        std::cout << prefix << std::endl;
      std::cout << (*this) << std::endl;
    }

    static Transform2Base identity()
    {
      return Transform2Base();
    }
  };

  template<typename T>
  std::ostream& operator<<(std::ostream& out, const Transform2Base<T>& m)
  {
    out << "translation:" << std::endl << m.translation << std::endl;
    out << "rotation:" << std::endl << m.rotation;
    return out;
  }

  template<typename T>
  Transform2Base<T> pose_update(const Transform2Base<T>& t1,
                                const Transform2Base<T>& t2)
  {
    return Transform2Base<T>(t1.translation + t1.rotation*t2.translation,
                             t1.rotation*t2.rotation);
  }

  template<typename T>
  Transform2Base<T> pose_inverse(const Transform2Base<T>& t)
  {
    return Transform2Base<T>(-1.0*t.rotation.trans()*t.translation,
                             t.rotation.trans());
  }

  template<typename T>
  Transform2Base<T> pose_delta(const Transform2Base<T>& t1,
                               const Transform2Base<T>& t2)
  {
    return Transform2Base<T>(t1.rotation.trans()*(t2.translation - t1.translation),
                             t1.rotation.trans()*t2.rotation);
  }

  typedef Transform2Base<float> Transform2f;
  typedef Transform2Base<double> Transform2d;
  typedef Transform2d Transform2;
  typedef Transform2 Tr2;
}
#endif
