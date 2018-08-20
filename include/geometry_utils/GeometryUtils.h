/*
  geometry_utils: Utility library to provide common geometry types and transformations
  Copyright (C) 2014  Nathan Michael

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include "Vector2.h"
#include "Vector3.h"
#include "Vector4.h"
#include "Quaternion.h"
#include "Matrix2x2.h"
#include "Matrix3x3.h"
#include "Matrix4x4.h"
#include "Rotation2.h"
#include "Rotation3.h"
#include "Transform2.h"
#include "Transform3.h"

namespace geometry_utils
{
  inline double unroll(double x)
  {
    x = fmod(x, 2.0*M_PI);
    if (x < 0) x += 2.0*M_PI;
    return x;
  }

  inline double normalize(double x)
  {
    x = fmod(x + M_PI, 2.0*M_PI);
    if (x < 0) x += 2.0*M_PI;
    return x - M_PI;
  }

  inline double shortest_angular_distance(double from, double to)
  {
    double result = unroll(unroll(to) - unroll(from));
    if (result > M_PI)
      result = -(2.0*M_PI - result);
    return normalize(result);
  }

  inline double rad2deg(double angle)
  {
    return angle*180.0*M_1_PI;
  }

  inline double deg2rad(double angle)
  {
    return angle*M_PI/180.0;
  }

  inline Vec3 rad2deg(const Vec3& angles)
  {
    return Vec3(rad2deg(angles(0)),
                rad2deg(angles(1)),
                rad2deg(angles(2)));
  }

  inline Vec3 deg2rad(const Vec3& angles)
  {
    return Vec3(deg2rad(angles(0)),
                deg2rad(angles(1)),
                deg2rad(angles(2)));
  }

  inline Vec3 RToZYX(const Rot3& rot)
  {
    return rot.getEulerZYX();
  }

  inline Rot3 ZYXToR(const Vec3& angles)
  {
    return Rot3(angles);
  }

  inline Rot3 QuatToR(const Quat& quat)
  {
    return Rot3(quat);
  }

  inline Quat RToQuat(const Rot3& rot)
  {
    return Quat(Eigen::Quaterniond(rot.eigen()));
  }

  inline double getRoll(const Rot3& r)
  {
    return r.roll();
  }

  inline double getRoll(const Quat& q)
  {
    return Rot3(q).roll();
  }

  inline double getPitch(const Rot3& r)
  {
    return r.pitch();
  }

  inline double getPitch(const Quat& q)
  {
    return Rot3(q).pitch();
  }

  inline double getYaw(const Rot3& r)
  {
    return r.yaw();
  }

  inline double getYaw(const Quat& q)
  {
    return Rot3(q).yaw();
  }

  inline double SO3Error(const Quat& q1, const Quat& q2)
  {
    return Rot3(q1).error(Rot3(q2));
  }

  inline double SO3Error(const Rot3& r1, const Rot3& r2)
  {
    return r1.error(r2);
  }

  namespace sys
  {
    inline Vec3 CartesianToSpherical(const Vec3& v)
    {
      double rho = v.norm();
      return Vec3(rho, acos(v.z()/rho), atan2(v.y(), v.x()));
    }

    inline Vec3 SphericalToCartesian(const Vec3& v)
    {
      return Vec3(v(0)*sin(v(1))*cos(v(2)),
                  v(0)*sin(v(1))*sin(v(2)),
                  v(0)*cos(v(1)));
    }

    inline Vec3 NEDCartesian(const Vec3& v)
    {
      return Vec3(v(0), -v(1), -v(2));
    }
  }
}
#endif
