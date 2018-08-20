#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "utils/parameters.h"
#include "utils/clock.h"
#include "disturbance/disturbance.h"
#include "vehicle_controller.h"

using namespace geometry_utils;

class AUVmodel
{

public:

  typedef boost::shared_ptr<AUVmodel> Ptr;
  typedef boost::shared_ptr<const AUVmodel> ConstPtr;

  AUVmodel(const utils::Parameters::Ptr&, const Disturbance::Ptr&);
  AUVmodel(const utils::Parameters::Ptr&, const Disturbance::Ptr&, const mdp_planner::MDP_Net::Ptr&);
  virtual ~AUVmodel();

  //load parameters from yaml
  void loadParams(void);

  //accessors
  void setVelApplied(const Vec2& in) { v_app = in; }
  //void getVelApplied(void) { return v_app;  }
  Vec2 getVelNet(void){ return v_app + v_disturb; }
  void settf2(const Transform2& in){ tf2 = in; }
  Transform2 gettf2(void) { return tf2; }
  AUVcontroller::Ptr getpControl(void) { return pControl; }

  //add gaussian noise to disturb vector
  void perturbDisturbance(Vec2&);

  //simple kinematics: given a net velocity, update next pose
  void modelDynamics(void);

  //vehicle model in environment 
  void poseEvolveInEnvironment(void); 

  //perturb vel with std, axis lable x, y, z, default all directions
  void perturbVel(Vec2& v, double noise_std, char axis_label='\0');


private:

  utils::Parameters::Ptr  	pParams;
  Disturbance::Ptr 		pDisturb;
  AUVcontroller::Ptr 		pControl;

  double m;					//mass
  Transform2 tf2;				//real vehicle tf2		

  double v_app_max;				//scalar, max vehicle applied velocity
  Vec2 v_app;					//read it from controller
 
  Vec2 v_disturb;				//real disturb

  /** auxiliary data structure **/

  //double dist_threshold;
  double disturb_timer;

};


#endif


