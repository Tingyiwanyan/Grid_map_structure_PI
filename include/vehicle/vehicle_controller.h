#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "utils/parameters.h"
#include "utils/clock.h"
#include "utils/functions.h"
#include "disturbance/disturbance.h"
#include "mdp_methods/mdp_net.h"
#include "mdp_methods/mdp_core.h"
#include "agent.h"

using namespace geometry_utils;


typedef
struct PID_Params {

  double k_p;
  double k_i;
  double k_d;

  PID_Params():k_p(0), k_i(0), k_d(0){}
  ~PID_Params(){}

} pid_params_t;



class AUVcontroller : public Agent
{

public:

  typedef boost::shared_ptr<AUVcontroller> Ptr;
  typedef boost::shared_ptr<const AUVcontroller> ConstPtr;

  AUVcontroller(const utils::Parameters::Ptr&);
  AUVcontroller(const utils::Parameters::Ptr&, const Disturbance::Ptr&, const mdp_planner::MDP_Net::Ptr&); //for MDP version
  virtual ~AUVcontroller();

  //load parameters from yaml
  void loadParams(void);
  void commonInit(void);

  //accessors
  mdp_planner::MDP_Net::Ptr  getpNet(void) { return pNet; }
  Vec2 getVelApplied(void) { return v_app; };
  void setCurTf2(const Transform2& in) { cur_tf2 = in; }
  Transform2 getCurTf2(void){ return cur_tf2; }
  void setLastTf2(const Transform2& in){ last_tf2 = in; }
  Transform2 getLastTf2(void){ return last_tf2; }
  bool stateIsChanged(void){ return pNet->getState(cur_tf2.translation) != pNet->getState(last_tf2.translation); }
  mdp_planner::mdp_state_t* getCurState(void){ return pNet->getState(cur_tf2.translation); }
  utils::Clock& getClock(void){ return clock; }
  bool getSimuStatus(void) { return simu_on; }
  bool getMotorStatus(void){ return motor_on; }

  //basic velocity controller
  void velocityController(const Vec2& v_ref);

  //basic position controller, currently not used as it's not needed
  void positionController(const pos2d_t& p_ref);


  /**********  MDP-action version of vehicle controller **********/

  // value iterations for non-shared policy for each vehicle
  void MDPIterations(const vector<Transform2>& tf2_goals);

  //mdp-acition controller
  void controllerMDP(void);


  /*********  Dead-Reckoning (DR) version of vehicle controller *********/

  //dead-reckoning controller
  void controllerDR(Transform2& _tf2_goal);

  //during the surfacing time, correct previous disturbance estimation, update ref tfs, etc
  void correctDeadReckoning(void);

   //interpolate a series surfacing waypoints between start-goal positions
  // interpolation segments parameterized in yaml, results stored in this->waypoints
  void interpolateWaypoints(const point2d_t& start, const point2d_t& goal);

   //given v_app_max and disturbance, get max net v at the specified net v direction
  Vec2 getMaxVelNetDR(const Vec2& net_v_dir, const Vec2& v_disturb_est);


  /**********  auxiliary functions  **********/

  //record waypoints to draw trajectory, can be space-expensive if traj is long
  void markTrajectory(void);
  
  //update next to-go reference pose along pre-generated waypoints, 
  // dist_delta: threshold for judging if have reached a waypoint
  void updateRefPoseWaypoints(double dist_threshold);  //un-used


private:

  utils::Parameters::Ptr  	pParams;
  Disturbance::Ptr              pDisturb;
  mdp_planner::MDP_Net::Ptr	pNet;		//the internal mdp net
  mdp_planner::MDP::Ptr         pMDP;		//the internal mdp policy

  double v_app_max;				//scalar, max vehicle applied velocity
  Vec2 v_ref, v_app;
 
  double ref_submerge_time, elapsed_submerge_time;

  Vec2 disturb_est;				//est: estimate/update in realtime

  double m;					//mass
  pid_params_t pid_v, pid_p;			//PID params of velocity and position

  int freq_traj_marks;				//the frequency of marking waypoints in trajectory 
  double segment_length;
  bool state_is_changed;			//under continueous motion, detect new discrete states

  //for stochastic shortest path
  std::deque<mdp_planner::mdp_state_t*> waystates;         //to-go way states (future)		

  utils::Clock clock;				// internal/local clock

  bool motor_on;                                //stop velocity control?
  bool simu_on;                                 //stop simulation?


  /** auxiliary data structure **/

  double traj_timer;
  bool pid_v_init, pid_p_init;
  Vec2 pid_v_i, pid_p_i;                   //integral of controllers (velocity and position)
  Vec2 pid_v_err, pid_p_err;               //last error to compute derivative
  mdp_planner::mdp_state_t* pState;        //last state, judge new state hopping moment (for new perturbation)


};


#endif


