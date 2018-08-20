
#include <random>
#include "disturbance/disturbance.h"


Disturbance::Disturbance(const utils::Parameters::Ptr& pParams, 
			 const mdp_planner::MDP_Grid2D::Ptr& pGrid2d){

  assert(pParams && pGrid2d);
  this->pParams = pParams;
  this->pGrid2d = pGrid2d;

  vec_field.resize(pGrid2d->cell_centers.size());
  swirl_center.set(0, 0); 

  loadParams();

  //force to udpate once
  updateDynamicVecField();

}

Disturbance::Disturbance(const utils::Parameters::Ptr& pParams, 
                         const utils::DataLoader::Ptr& pData, 
                         const mdp_planner::MDP_Grid2D::Ptr& pGrid2d){

  assert(pParams && pData && pGrid2d);
  this->pParams = pParams;
  this->pData = pData;
  this->pGrid2d = pGrid2d;

  vec_field.resize(pGrid2d->cell_centers.size());
  if(pParams->getParamNode()["method_manager"].as<string>().compare("imported_vf"))
    updateDynamicVecFieldNC();

}

Disturbance::Disturbance(const utils::Parameters::Ptr& pParams, const vector<Vec2>& vf){

  vec_field = vf;

}

Disturbance::~Disturbance(){}


void
Disturbance::loadParams(void){

  v_uniform.x()	= pParams->getParamNode()["disturbance"]["v_uniform"]["x"].as<double>();
  v_uniform.y()	= pParams->getParamNode()["disturbance"]["v_uniform"]["y"].as<double>();

/*
  //check if it is a dynamic vf
  dynamic_vf = false;
  if(pParams->getParamNode()["disturbance"]["translating_center"].as<bool>() || 
	 pParams->getParamNode()["disturbance"]["rotating_vectors"].as<bool>() ){
    dynamic_vf = true;
  }
*/

}


vector<Vec2>
Disturbance::getVecField(uint frame) {
  unsigned int n = pData->getNCdata().n_rows * pData->getNCdata().n_cols;
  vector<Vec2> vec(n);
  for (uint i = 0; i < pData->getNCdata().n_rows; i++) {
    for (uint j = 0; j < pData->getNCdata().n_cols; j++) {
      vec[i * pData->getNCdata().n_cols + j] = pData->getNCdata().frames[frame].cells[i][j].ocean_vec;
    }
  }
  return vec;

}


Vec2
Disturbance::getVec(uint id){

  return vec_field[id];
 
}

/*
Vec2
Disturbance::getVec(const mdp_planner::mdp_state_t* s){

  return getVec(s->id);

}


Vec2
Disturbance::getVec(const point2d_t& p){

  assert(pGrid2d);
  mdp_planner::mdp_state_t* s = pGrid2d->getState(p);
  return getVec(s);

}
*/


Vec2
Disturbance::predictDisturbanceNC(const point2d_t& pt, double t){

  Vec2 v_d;

  int idx = pGrid2d->getID(pt);
  v_d.x() = pData->getuGPs()[idx]->f(&t);
  v_d.y() = pData->getvGPs()[idx]->f(&t);

  double rescale_vf = pParams->getParamNode()["imported_vf"]["rescale_vf"].as<double>();
  v_d.scale(rescale_vf);

  return v_d;
}


Vec2
Disturbance::predictDisturbance(const point2d_t& pt, double t){

  string key 		= pParams->getParamNode()["disturbance"]["pattern"].as<string>();
  bool is_translating	= pParams->getParamNode()["disturbance"]["translating_center"].as<bool>();
  bool is_rotating	= pParams->getParamNode()["disturbance"]["rotating_vectors"].as<bool>();

  // center is translated by swirl_center, trajectory is an ellipse
  if(is_translating){
    double major = 20; 
    double minor = 5;
    double rotate_rate = 20*M_PI/180;
    double ellipse_angle = rotate_rate * t;
    swirl_center = Vec2(major*cos(ellipse_angle), minor*sin(ellipse_angle));
    //rotate the ellipse so that the major is along grid map diagonal
    geometry_utils::Rot2 rot(M_PI/4);
    swirl_center = rot*swirl_center;
  }

  Vec2 v_d;

  //f(x, y) = [1, 1] 		//uniform
  if(key.compare("uniform")==0){
    v_d = v_uniform;
  }

  // f(x, y) = [-x -y] 	//centripetal
  if(key.compare("centripetal")==0){
    v_d.set( - (pt.x()- swirl_center.x()), - (pt.y() - swirl_center.y()));
  }

  // f(x, y) = [(-x+y) (-x-y)] //swirl
  if(key.compare("swirl")==0){
    v_d.set( -(pt.x()-swirl_center.x()) + (pt.y()-swirl_center.y()), - (pt.x()-swirl_center.x()) - (pt.y()-swirl_center.y()) );
  }

  // f(x, y) = [y (-x+y)] 	//swirl
  if(key.compare("swirl-y")==0){
    v_d.set( (pt.y()-swirl_center.y()), -(pt.x()-swirl_center.x()) + (pt.y()-swirl_center.y()) );
  }

  // f(x, y) = [-x (-x-y)]	//swirl
  if(key.compare("swirl-x")==0){
    v_d.set( -(pt.x()-swirl_center.x()), - (pt.x()-swirl_center.x()) - (pt.y()-swirl_center.y()) );
  }

  // f(x, y) = [x (x-y)], 	//hyperbolic
  if(key.compare("hyperbolic")==0){
    v_d.set( (pt.x()-swirl_center.x()), (pt.x()-swirl_center.x()) - (pt.y()-swirl_center.y()) );
  }

  // update the rotating angle for each of vector in the field: a = omega*dt
  if(is_rotating){
    geometry_utils::Rot2 rot2;
    double rotate_omega = pParams->getParamNode()["disturbance"]["rotate_omega"].as<double>();
    double vector_angle = rotate_omega * t * M_PI/180.;
    rot2.fromAngle(vector_angle);
    v_d = rot2*v_d;
  }
 
  if(key.compare("uniform") !=0 ){
    double max_length = pParams->getParamNode()["disturbance"]["max_magnitude"].as<double>();
    if(v_d.norm() > max_length){
      double re_scale = max_length/v_d.norm();
      v_d.scale(re_scale);
    }
  }

  return v_d;

}


void
Disturbance::updateDynamicVecField(void){

  //wallClock;
  double wall_clock_t = utils::WallClock::getWallTime();
  if(!utils::WallClock::dtIsSet())
    wall_clock_t = 0; 

  for(uint i=0; i<vec_field.size(); i++){
    point2d_t pt = pGrid2d->cell_centers[i];
    vec_field[i] = predictDisturbance(pt, wall_clock_t);
  }

}


void
Disturbance::updateDynamicVecFieldNC(void){

  //wallClock;
  double wall_clock_t = utils::WallClock::getWallTime();
  if(!utils::WallClock::dtIsSet())
    wall_clock_t = 0; 

  double time_scale = pParams->getParamNode()["imported_vf"]["rescale_nc_interval"].as<double>();
  for(uint i=0; i<vec_field.size(); i++){
    point2d_t pt = pGrid2d->cell_centers[i];
    vec_field[i] = predictDisturbanceNC(pt, wall_clock_t/time_scale);
  }

}


//stream function used in meanderVecField
double
stream_func(double t, double x, double y){

  //parameters from Mike Eichhorn's paper
  double B0 	= 1.2*5;	//adjust amplitude
  double epsilon= 0.3;
  double omega 	= 0.4;
  double theta	= M_PI/2;

  double k	= 0.84*0.4;
  double c	= 0.12*10;

  //oscillation function
  double B_t = B0 + epsilon*cos(omega*t + theta);

  //stream function
  double phi_xy = 1 - tanh( (y-B_t*cos(k*(x-c*t))) /
		(sqrt(1+k*k*B_t*B_t*sin(k*(x-c*t)*sin(k*(x-c*t))))) );

  return phi_xy;

}


//not used
void
Disturbance::meanderVecField(void){

  //double t = getCurTime();
  if(!utils::WallClock::dtIsSet())
    return;
  double dt = utils::WallClock::dtIsSet();
  double t  = utils::WallClock::getRealTime();
  double dx = 1e-7, dy = 1e-7;

  for(uint i=0; i<vec_field.size(); i++){
    point2d_t pt = pGrid2d->cell_centers[i];
    double x_component = (stream_func(t, pt.x(), pt.y()+dy) - stream_func(t, pt.x(), pt.y()))/dy;
    double y_component = (stream_func(t, pt.x()+dx, pt.y()) - stream_func(t, pt.x(), pt.y()))/dx;
    vec_field[i].set(x_component, y_component);
    double scale = 100;
    vec_field[i].scale(scale);
  }

}



