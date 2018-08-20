
// basic point, pos, and pose
// lantao

#ifndef POSE_H
#define POSE_H

#include "Vector3.h"
#include "Vector2.h"
#include "Quaternion.h"

namespace gu = geometry_utils;

typedef gu::Vector3d point_t;	//3d point defult
typedef point_t pos_t;  	//define position/translation as 3d vec

typedef gu::Vector2d point2d_t;	//2d point used in grid2d
typedef point2d_t pos2d_t;  	//define position/translation as 3d vec


// Pose is no longer used 12/8/2015
typedef
struct Pose {

  pos_t pos;                  //3d position
  double roll, pitch, yaw;    //

  Pose():roll(0),pitch(0),yaw(0){ pos.set(0, 0, 0);}
  Pose(double _x, double _y, double _z){ pos.set(_x, _y, _z); }
  Pose(point_t& p):pos(p){}
  ~Pose(){}

  point_t getPoint(void) const {return pos; }
  void set(const Pose& p){
    this->pos=p.pos; this->roll=p.roll; this->pitch=p.pitch; this->yaw=p.yaw;
  }
  void setPos(double _x, double _y, double _z){ pos.set(_x, _y, _z); }
  void setPos(const pos_t& _pos){ this->pos = _pos; }

} pose_t;


/* Build a unit quaternion representing the rotation
 * from u to v. The input vectors need not be normalised. 
 * http://lolengine.net/blog/2014/02/24/quaternion-from-two-vectors-final
*/
inline gu::Quat
getQuaternion(const gu::Vec3& u, const gu::Vec3& v){

  double norm_uv = sqrt(u.dot(u) * v.dot(v));
  double real_part = norm_uv + u.dot(v);
  gu::Vec3 w;

  if(real_part < 1.e-6f * norm_uv)
  {
    /* If u and v are exactly opposite, rotate 180 degrees
     * around an arbitrary orthogonal axis. Axis normalisation
     * can happen later, when we normalise the quaternion. */
    real_part = 0.0f;
    w = abs(u.x()) > abs(u.z()) ? gu::Vec3(-u.y(), u.x(), 0.)
			    : gu::Vec3(0., -u.z(), u.y());
  }
  else
  {
    /* Otherwise, build quaternion the standard way. */
    w = u.cross(v);
  }

  gu::Quat q(real_part, w.x(), w.y(), w.z());
  q.normalize();
  return q;

}


#endif
