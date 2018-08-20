
#include "vehicle/vehicle_controller.h"


AUVcontroller::AUVcontroller(const utils::Parameters::Ptr& pParams){

  assert(pParams);
  this->pParams  = pParams;

  loadParams();
  commonInit();

}

AUVcontroller::AUVcontroller(const utils::Parameters::Ptr& pParams, const Disturbance::Ptr& pDisturb, const mdp_planner::MDP_Net::Ptr& pNet) {

  assert(pParams);
  this->pParams  = pParams;
  assert(pDisturb);
  this->pDisturb = pDisturb;
  assert(pNet);
  if(pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>())
    this->pNet = pNet;
  else
    this->pNet = mdp_planner::MDP_Net::Ptr(new mdp_planner::MDP_Net(pNet));

  loadParams();
  commonInit();

}

AUVcontroller::~AUVcontroller(void){}


void
AUVcontroller::loadParams(void){

  v_app_max	= pParams->getParamNode()["vehicle_model"]["v_max"].as<double>();
  m		= pParams->getParamNode()["vehicle_model"]["mass"].as<double>();
  pid_v.k_p	= pParams->getParamNode()["vehicle_controller"]["PID_velocity"]["k_p"].as<double>();
  pid_v.k_d	= pParams->getParamNode()["vehicle_controller"]["PID_velocity"]["k_d"].as<double>();

/*
  tf2_start.translation.x()  = pParams->getParamNode()["vehicle_model"]["start_tf2"]["x"].as<double>();
  tf2_start.translation.y()  = pParams->getParamNode()["vehicle_model"]["start_tf2"]["y"].as<double>();
  tf2_start.rotation.fromAngle(pParams->getParamNode()["vehicle_model"]["start_tf2"]["a"].as<double>());
  cur_tf2  = tf2_start;
  last_tf2 = cur_tf2;

  tf2_goal.translation.x()  = pParams->getParamNode()["vehicle_controller"]["goal_tf2"]["x"].as<double>();
  tf2_goal.translation.y()  = pParams->getParamNode()["vehicle_controller"]["goal_tf2"]["y"].as<double>();
  tf2_goal.rotation.fromAngle(pParams->getParamNode()["vehicle_controller"]["goal_tf2"]["a"].as<double>());
*/

  freq_traj_marks = pParams->getParamNode()["vehicle_controller"]["freq_traj_marks"].as<int>();
  segment_length  = pParams->getParamNode()["dead_reckoning"]["segment_length"].as<double>();
  //dist_threshold  = pParams->getParamNode()["dead_reckoning"]["dist_threshold"].as<double>();

  if(pParams->getParamNode()["macro_controller"].as<string>().compare("dead_reckoning")==0)
    cout<<"Dead reckoning controller is selected."<<endl;
  if(pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy")==0)
    cout<<"MDP policy controller is selected."<<endl;
 
}


void
AUVcontroller::commonInit(void){

  //init params
  waypoints.clear();
  disturb_est.set(0, 0);
  v_app.set(0, 0);
  v_ref.set(0, 0);
  ref_submerge_time = 0;

  state_is_changed = true;

  motor_on = true;
  simu_on  = true;
 
  traj_timer = 0;
  pid_v_init = false;
  pid_p_init = false;

  //init states
  //pNet->getState(tf2_goal.translation)->type = GOAL;
  //pNet->getState(tf2_start.translation)->type = START;

}


void
AUVcontroller::velocityController(const Vec2& v_ref){

  //PID
  if( (v_ref - v_app ).norm() > 1e-2){
    Vec2 v_err = v_ref - v_app;
    Vec2 err = v_err/m; 	//input error as acceleration err

    pid_v_i += err;
    Vec2 pid_v_d = err - pid_v_err;
    pid_v_err = err;

    if(!pid_v_init){
      pid_v_init = true;
      return;
    }
    //acceleration as control input
    Vec2 a = pid_v.k_p*err + pid_v.k_i*pid_v_i + pid_v.k_d*pid_v_d;
    //make sure dt is meaningful (initialized in clock)
    if(!utils::WallClock::dtIsSet())
      return;
    v_app = v_app + clock.getdt() * a;
   
    //saturate velocity if too large
    if(v_app.norm() > 5*v_app_max){
      v_app.scale(5*v_app_max/v_app.norm());
      //cout<<"dt: "<<clock.getdt()<<" a: "<<a<<" v_app: "<<v_app<<endl;
      cout<<"Too large applied velocity, saturated."<<endl; 
    }
  }

}


void
AUVcontroller::positionController(const pos2d_t& p_ref){

  //PID
  if( (p_ref - cur_tf2.translation ).norm() > 1e-1){
    Vec2 p_err = p_ref - cur_tf2.translation;
    Vec2 err = p_err/1.0; 	//input error as "velocity err"

    pid_p_i += err;
    Vec2 pid_p_d = err - pid_p_err;
    pid_p_err = err;

    if(!pid_p_init){
      pid_p_init = true;
      return;
    }
    //acceleration as control input
    Vec2 v = pid_p.k_p*err + pid_p.k_i*pid_p_i + pid_p.k_d*pid_p_d;
    //make sure dt is meaningful (initialized in clock)
    if(!utils::WallClock::dtIsSet())
      return;
    cur_ref_tf2 = cur_tf2 + clock.getdt() * v;
  }

}



void
AUVcontroller::MDPIterations(const vector<Transform2>& tf2_goals){
  
  using namespace mdp_planner;

  pMDP = MDP::Ptr(new MDP(pParams, pNet, pDisturb));

  // set the state type
  for(uint i=0; i<tf2_goals.size(); i++){
    double goal_value = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
    pNet->getState(tf2_goals[i].translation)->type = GOAL;
    pMDP->fillTypeValue(pNet->getState(tf2_goals[i].translation), GOAL, goal_value);
  }
  
  pMDP->iterations();

}



void
AUVcontroller::controllerMDP(void){

  clock.runClock();

  mdp_planner::mdp_state_t* s = pNet->getState(cur_tf2.translation);
  Vec2 v_act_dir = s->getOptimalActionVector();

  //perturb action with artificial noise
  static double angle_perturb;
  if(s != pState || !pState){
    double sigma =  pParams->getParamNode()["vehicle_model"]["gaussian_sigma"].as<double>();
    angle_perturb = utils::generateGaussianNoise(0, sigma);
    geometry_utils::Rot2 rot2(angle_perturb);
    v_act_dir = rot2*v_act_dir;
    pState = s;
    //cout<<"vehicle velocity perturbation angle: "<<angle_perturb*180/M_PI<<endl;
  }

  v_ref = v_app_max * v_act_dir; 

  velocityController(v_ref);
  
}



void
AUVcontroller::controllerDR(Transform2& _tf2_goal){

  clock.runClock();

  if(waypoints.empty() && fabs(ref_submerge_time) < EPSILON /*the init condition*/){
    interpolateWaypoints(cur_tf2.translation, _tf2_goal.translation);
    cout<<"Interpolated "<<waypoints.size()<<" waypoints."<<endl;
    cur_ref_tf2 = waypoints.front();
  }

  if(waypoints.empty()) 
    return;

  correctDeadReckoning();

  velocityController(this->v_ref);

}


void
AUVcontroller::correctDeadReckoning(void){
  
  // surfacing moment
  if(elapsed_submerge_time > ref_submerge_time /*timeout for surfacing*/
	|| fabs(ref_submerge_time) < EPSILON /*init condition*/){

    //if first iteration at 1st waypoint (current pos), init last_ref_tf2
    if(fabs(ref_submerge_time) < EPSILON && !waypoints.empty()){
      last_ref_tf2 = waypoints.front();
    }

    // waypoints contains the starting/current tf
    if(!waypoints.empty())
      waypoints.pop_front();

    if(!waypoints.empty()){
      cur_ref_tf2 = waypoints.front();
      cout<<endl<<"moving to next pose : "<<cur_ref_tf2.translation<<endl;
    }
    else{
      cout<<endl<<"finished all waypoints!"<<endl;
      simu_on = false;
      return;
    }

    //update vel based on cur tf and cur ref tf
    Vec2 d_err = cur_tf2.translation - last_ref_tf2.translation;

    //estimate disturbance starting from the 1st submerging time (2nd waypoint)
    if(fabs(ref_submerge_time) > EPSILON){
      Vec2 disturb_est_net = d_err/ref_submerge_time;
      disturb_est += disturb_est_net;  	//accumulated disturbance est
    }

    //update v_ref
    Vec2 d_dir = cur_ref_tf2.translation - cur_tf2.translation;
    Vec2 v_net_est = getMaxVelNetDR(d_dir, disturb_est);  
    v_ref = v_net_est - disturb_est;
    assert(v_ref.norm() < v_app_max + EPSILON);

    cout<<"     distance error : "<<d_err<<endl;
    cout<<"   disturb estimate : "<<disturb_est<<endl;
    cout<<"   vel net estimate : "<<v_net_est<<endl;

    //re-estimate ref_sub time and re-init elapsed time
    //TODO: estimated ref sub time is coarse here as the kinematics contain acceleration
    // currently we don't need be accurate, otherwise need to generate noise for disturbance
    ref_submerge_time = (cur_ref_tf2.translation - cur_tf2.translation).norm()/v_net_est.norm();
    elapsed_submerge_time = 0;

    last_ref_tf2 = cur_ref_tf2;

  }

  if(!utils::WallClock::dtIsSet())  return;
  double dt = clock.getdt();
  elapsed_submerge_time += dt;
  //cout<<"ref pos : "<<cur_ref_tf2.translation<<" cur pos: "<<cur_tf2.translation<<endl;
  //cout<<"ref time: "<<ref_submerge_time<<" elapsed time: "<<elapsed_submerge_time<<endl;
 
}


void
AUVcontroller::interpolateWaypoints(const point2d_t& start, const point2d_t& goal){

  waypoints.clear();
  Vec2 vec = goal - start;
  
  point2d_t p = start;
  waypoints.push_back(p); 	//put the starting node, can be useful for some conditions

  while( (p - goal).norm() > segment_length){
    p += segment_length * vec / vec.norm(); 
    waypoints.push_back(p);
  } 

  waypoints.push_back(goal);

}


Vec2
AUVcontroller::getMaxVelNetDR(const Vec2& v_net_dir, const Vec2& v_disturb_est){

  //for v_net_dir, only the direction is useful
  Vec2 v2_n(v_net_dir);		//2d net velocity
  Vec2 v2_d(v_disturb_est);	//2d disturbance vel

  if(v2_d.norm() < EPSILON){
    return v_app_max*v2_n/v2_n.norm();
  }

  // math: given two sides (one of them with unknown length) and their inner angle, ask for the 3d side
  // get inner angle (angle1) between v2_d and v2_n
  double cos_theta = v2_d.dot(v2_n)/(v2_d.norm()*v2_n.norm()) ;
  double angle1 = acos(std::min(std::max(cos_theta, -1.0), 1.0));	//acos returns NaN if out [-1, 1]

  if( (angle1<=M_PI/2 && v2_d.norm()*sin(angle1) >= v_app_max ) || 
	(angle1>=M_PI/2 && v2_d.norm() >= v_app_max)){
    cerr<<"steering component of vehicle velocity < disturbance velocity, un-controllable."<<endl;
    if(angle1<=M_PI/2 && v2_d.norm()*sin(angle1) >= v_app_max ){
      double v_d_on_net_len = v2_d.norm()*cos(angle1);
      Vec2 v_d_on_net = v_d_on_net_len * v2_n / v2_n.norm();
      return v_d_on_net - v2_d;
    }
    if(angle1>=M_PI/2 && v2_d.norm() >= v_app_max){
      return -v2_d;
    }
  }

  //law of sines, get angle (angle2) facing to v2_d: a/sin(A) = b/sin(B)
  //note: since sin x = sin (pi-x), so there will be two solutions, but since we want the 3rd angle as large as possible, so the default acute angle solution from asin is what we need 
  double sin_theta = v2_d.norm()*sin(angle1)/v_app_max;
  double angle2 = asin(std::min(std::max(sin_theta, -1.0), 1.0));   //asin returns NaN if out [-1, 1]
  double angle3 = M_PI - angle1 - angle2;
  assert(angle3+EPSILON>0 && angle3-EPSILON<M_PI);
  
  //law of cosines get the 3rd side: c^2 = a^2 + b^2 - 2abcos(C)
  double length = sqrt( v2_d.norm()*v2_d.norm() + v_app_max*v_app_max - 
		  2*v2_d.norm()*v_app_max*cos(angle3) );

  return length*v2_n/v2_n.norm();

}


//tested but un-used (waypoints ground truth is assumed unknown)
void
AUVcontroller::updateRefPoseWaypoints(double _dist_threshold){

  while(!waypoints.empty() && 
	      (waypoints.front().translation - cur_tf2.translation).norm() < _dist_threshold){
    waypoints.pop_front();
    if(!waypoints.empty()){
      last_ref_tf2 = cur_ref_tf2;
      cur_ref_tf2 = waypoints.front();
      cout<<"moving to next pose: "<<cur_ref_tf2.translation<<endl;
    }
    else{
      cout<<"finished all waypoints!"<<endl;
    }
  }

}


void
AUVcontroller::markTrajectory(void){

  if(!utils::WallClock::dtIsSet())
    return;

  double period = 1.0/freq_traj_marks;

  traj_timer += utils::WallClock::getdt();
  //currently record only 1k poses
  if(traj_timer > period && trajectory.size()<1e4){
    trajectory.push_back(cur_tf2);
    traj_timer = 0;
  }

}



