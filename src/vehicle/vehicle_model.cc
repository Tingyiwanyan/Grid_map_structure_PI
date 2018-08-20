
#include <random>
#include "vehicle/vehicle_model.h"


AUVmodel::AUVmodel(const utils::Parameters::Ptr& pParams, const Disturbance::Ptr& pDisturb){

  assert(pParams);
  assert(pDisturb);
  this->pParams  = pParams;
  this->pDisturb = pDisturb;

  this->pControl = AUVcontroller::Ptr(new AUVcontroller(pParams));

  v_disturb.set(0, 0);

  loadParams();

}

AUVmodel::AUVmodel(const utils::Parameters::Ptr& pParams, const Disturbance::Ptr& pDisturb, const mdp_planner::MDP_Net::Ptr& pNet){

  assert(pParams);
  assert(pDisturb);
  assert(pNet);
  this->pParams  = pParams;
  this->pDisturb = pDisturb;

  this->pControl = AUVcontroller::Ptr(new AUVcontroller(pParams, pDisturb, pNet));

  v_disturb.set(0, 0);

  disturb_timer = 0;

  loadParams();

}

AUVmodel::~AUVmodel(void){}


void
AUVmodel::loadParams(void){

  v_app_max = pParams->getParamNode()["vehicle_model"]["v_max"].as<double>();
  m	    = pParams->getParamNode()["vehicle_model"]["mass"].as<double>();

/*
  tf2.translation.x()  = pParams->getParamNode()["vehicle_model"]["start_tf2"]["x"].as<double>();
  tf2.translation.y()  = pParams->getParamNode()["vehicle_model"]["start_tf2"]["y"].as<double>();
  tf2.rotation.fromAngle(pParams->getParamNode()["vehicle_model"]["start_tf2"]["a"].as<double>());
*/

}


void 
AUVmodel::perturbDisturbance(Vec2& disturb){

  //perturb the disturbance with artificial noise
  static double angle_perturb;
  double sigma  = pParams->getParamNode()["disturbance"]["gaussian_sigma"].as<double>();
  int freq	= pParams->getParamNode()["disturbance"]["perturb_freq"].as<int>();
  if(freq == 0)
    return;

  if(!utils::WallClock::dtIsSet())  return;
  disturb_timer += utils::WallClock::getdt();

  double period = 1.0/freq;
  if(disturb_timer > period){
    angle_perturb = utils::generateGaussianNoise(0, sigma);
    geometry_utils::Rot2 rot2(angle_perturb);
    v_disturb = rot2*v_disturb;
    disturb_timer = 0;
    //cout<<"disturbance perturbation angle: "<<angle_perturb*180/M_PI<<endl;
  }

}


void 
AUVmodel::modelDynamics(void){

  if(!pControl->getSimuStatus())
    return;

  uint vf_id = pControl->getpNet()->getState(tf2.translation)->id; 
  v_disturb = pDisturb->getVec(vf_id);
  if(std::isnan(v_disturb.x()) || std::isnan(v_disturb.y()) ){
    cerr<<"the queried grid has no vec field! Possibly in obstacle state!"<<endl<<endl;
    exit(0);
  }
  Vec2 v_disturb0(v_disturb); //save un-disturbed version

  //add noise
  perturbDisturbance(v_disturb);

  v_app = pControl->getVelApplied();

  if(!pControl->getMotorStatus())
    v_app.set(0, 0);

  Vec2 v_net = v_app + v_disturb;

  if(!utils::WallClock::dtIsSet())
    return;

  //update pose based on simple kinematics instead of dynamics. Improve it later
  double dt = utils::WallClock::getdt();
  tf2.translation += dt * v_net;
  //tf2.rotation.fromAngle(std::atan2(v_net.y(), v_net.x()));  // shaky orientation due to noise
  Vec2 v_net0 = v_app + v_disturb0;
  tf2.rotation.fromAngle(std::atan2(v_net0.y(), v_net0.x()));  
  
}


//vehicle model, evolved in environment
void 
AUVmodel::poseEvolveInEnvironment(void){

  //terminate condition
  if(pControl->getpNet()->getState(tf2.translation)->type==GOAL){
    //cout<<"goal state reached!"<<endl;
    return;     
  }

  if(pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy")==0){
    //document cur tf2 as last
    pControl->setLastTf2(pControl->getCurTf2());
    //updated cur tf2 resulted from environment dyanmics, closed-loop/feedback
    pControl->setCurTf2(this->gettf2());
    pControl->controllerMDP();
  }
  else if(pParams->getParamNode()["macro_controller"].as<string>().compare("dead_reckoning")==0){
    pControl->setLastTf2(pControl->getCurTf2());
    pControl->setCurTf2(this->gettf2());
    //Note: currently here the goal is fixed as g0!!
    vector<double> v = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as<vector<double> >();
    Transform2 g_tf2(v[0], v[1], v[2]);
    pControl->controllerDR(g_tf2);
  }
  else{
    cerr<<"!!Please select a controller."<<endl;
    assert(0);
  }

  modelDynamics();
  pControl->markTrajectory();

}


void
AUVmodel::perturbVel(Vec2& v, double noise_std, char axis_label){

  std::default_random_engine generator(utils::WallClock::getRealTime()*1e6);
  std::normal_distribution<double> distribution(0.0, noise_std);

  Vec2 noise(distribution(generator), 
	     distribution(generator));
  cout<<"noise: "<<noise<<endl;

  switch (axis_label) {
    case '\0': v += noise;  break;
    case 'x': v.x() += noise.x(); break;
    case 'y': v.y() += noise.y(); break;
    //case 'z': v.z() += noise.z(); break;
    default: assert(0);
  }

}


