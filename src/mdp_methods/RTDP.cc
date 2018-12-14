#include "geometry_utils/Vector4.h"
#include "mdp_methods/mdp_core.h"
#include "method_manager/method_manager.h"
#include "mdp_methods/ssp.h"
#include "method_manager/method_manager.h"
#include "animation/animation.h"
#include "mdp_methods/ssa.h"
#include "mdp_methods/mdp_state.h"
#include "geometry_utils/Vector2.h"
#include <map>
#include <queue>
#include<ctime>
#include<cstdlib>
#include<math.h>
#include<vector>
#include<iostream>
#include<fstream>

using namespace geometry_utils;
using namespace mdp_planner;

void mdp_planner::getTransitionModel(mdp_state_t* s, action_t act,MDP_Net::Ptr& pNet)
{
  //assert(s->successors[act]!=s);

  double v_max = 1;//pParams->getParamNode()["vehicle_model"]["v_max"].as<double>();
  double sigma = 1;//pParams->getParamNode()["vehicle_model"]["gaussian_sigma"].as<double>();
  //double sigma_d = pParams->getParamNode()["disturbance"]["gaussian_sigma"].as<double>();
  //bool back_transition = pParams->getParamNode()["mdp_methods"]["transition_model"]["back_transition"].as<bool>();

  //get the unit applied vec for action a
  Vec2 v_app = mdp_state_t::getActionVector(act);
  v_app = v_max*v_app;
  Vec2 v_net = v_app;

  //each prob is associated with a action/transition link
  s->probs.resize(NUM_ACTIONS, 0);

  double sum = 0;
  for(uint i=NORTH; i<NUM_ACTIONS; i++){
      if(s->successors[i]->id != s->id){
          Vec2 v_succ = pNet->cell_centers[s->successors[i]->id] -
                  pNet->cell_centers[s->id];
          double theta, cos_theta;
          if(v_net.norm() > EPSILON){
              cos_theta = v_net.dot(v_succ)/(v_net.norm()*v_succ.norm());
              theta = std::acos(std::min(std::max(cos_theta, -1.0), 1.0));
              //method1: using gaussian pdf to emulate gaussian distribution on a polar coordinate
              s->probs[i] = utils::gaussian_pdf(theta, 0, sigma);
              //method2: using cosine curve to emulate gaussian like distribution with p\in[0, 2]
              //s->probs[i] = (cos_theta+1);
          }
          else{
              theta = 0; sigma = 1e+2;
              //large sigma generates pretty much like "uniform" distriubtion
              s->probs[i] = utils::gaussian_pdf(theta, 0, sigma);
          }
          //if only forward transitions are possible, set reverse transitions 0
          if(cos_theta < 0){
              s->probs[i] = 0;
          }
      }
      else{
        if(s->successors[act]->id == s->id)
          {s->probs[i] = 0.9;}
          else
          {
            s->probs[i] = 0.1;
          }
        }

      sum += s->probs[i];
  }

  //normalize to prob
  for(uint i=NORTH; i<NUM_ACTIONS; i++)
      s->probs[i] = (sum<EPSILON) ? 0 : s->probs[i]/sum;
}

void mdp_planner::uniform_transition_initialization(MDP_Net::Ptr& pNet)
{
  std::cout<<"IM here at uniform"<<std::endl;
  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    if(pNet->mdp_states[i]->type != OBSTACLE){
      for(uint j=NORTH; j<NUM_ACTIONS; j++){
        pNet->mdp_states[i]->probs[j]=1.0/9.0;
        pNet->mdp_states[i]->policy[j]=1.0/9.0;
        //std::cout<<pNet->mdp_states[i]->probs[j]<<std::endl;
      }
    }
  }
}


void mdp_planner::Policy_evaluation(MDP_Net::Ptr& pNet)
{
  //mdp_planner::uniform_transition_initialization(pNet);
  double final_value = 0;
  double error;
  double error_final = INF;
  while(error_final > 0.5)
  //for(int kk=0;kk<1;kk++)
  {
    error_final = 0;
    for(int i=0;i<pNet->mdp_states.size();i++)
    {
      final_value = 0;
      mdp_state_t *s = pNet->mdp_states[i];
      if(s->type == BODY || s->type == START)
      {
      for(uint j=NORTH; j<NUM_ACTIONS; j++)
      {
        action_t act = (action_t)j;
        //getTransitionModel(s, act,pNet);
        final_value += s->policy[j]*mdp_planner::addSuccessorValue(s,act,pNet);
      }
      error = std::abs(final_value-s->optimal_value);
      s->optimal_value = final_value;
      if(error >error_final )
      {
        error_final = error;
      }

    }
    }
    for(int i=0;i<pNet->mdp_states.size();i++)
    {
      mdp_state_t *s = pNet->mdp_states[i];
      s->last_optimal_value = s->optimal_value;
    }
/*
    std::cout << "\nmap indices" << std::endl;
    for (int i = pNet -> n_rows - 1; i >= 0; i--) {
        for (int j = 0; j < pNet -> n_cols; j++) {
            //std::cout << i * pGrid2d -> n_cols + j << " ";
            std::cout<< (int)pNet->mdp_states[i*pNet->n_cols+j]->optimal_value<<" ";
            //my_file2<<pNet->mdp_states[i*pGrid2d->n_cols+j]->optimal_value<<",";
        }
        std::cout << std::endl;
    }
    */

  //std::cout<<"Error_final "<<error_final<<endl;
  }
}

void mdp_planner::Policy_update(MDP_Net::Ptr& pNet)
{
  double final_value = 0;
  double value_current = 0;
  action_t best_act=(action_t)1;
  int index;
  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    if(pNet->mdp_states[i]->type == BODY || pNet->mdp_states[i]->type == START){
    pNet->mdp_states[i]->last_optimal_action = pNet->mdp_states[i]->optimal_action;
  }
  }
  for(int i = 0;i<pNet->mdp_states.size();i++)
  {
    index = 0;
    final_value = -INF;
    mdp_state_t *iter_state = pNet->mdp_states[i];

    if(iter_state->type == BODY || iter_state->type == START)
    {

    for(uint j=NORTH; j<NUM_ACTIONS; j++)
    {
      action_t act = (action_t)j;
      value_current = mdp_planner::addSuccessorValue(iter_state,act,pNet);
      if(value_current > final_value)
      {
        final_value = value_current;
        best_act = act;
        index = j;
      }
    }

    iter_state->optimal_action = best_act;


    action_t opt_act = iter_state->optimal_action;
    for(uint k=NORTH; k<NUM_ACTIONS; k++){
      iter_state->policy[k]=0;
      //std::cout<<pNet->mdp_states[i]->probs[j]<<std::endl;
    }
    //std::cout<<"best_act "<<best_act<<std::endl;
    iter_state->policy[best_act]=1.0;


    for(uint l=NORTH;l<NUM_ACTIONS;l++)
    {
      action_t act_curr = (action_t)l;
      iter_state->actions[act_curr]=false;
    }

    iter_state->actions[best_act]=true;

  }
  }
}

void mdp_planner::Policy_iteration(MDP_Net::Ptr& pNet)
{
  int num_iteration=0;
  mdp_planner::Policy_evaluation(pNet);
  mdp_planner::Policy_update(pNet);
  double average_value=0;
  while(mdp_planner::check_policy_converge_PI(pNet->mdp_states))
  //for(int k=0;k<15;k++)
  {
    average_value = 0;
    mdp_planner::Policy_evaluation(pNet);
    mdp_planner::Policy_update(pNet);
    for(int i = 0;i<pNet->mdp_states.size();i++)
    {
      average_value+=pNet->mdp_states[i]->optimal_value;
    }
    average_value = average_value/pNet->mdp_states.size();
    num_iteration++;
    std::cout<<"iternumber "<<num_iteration<<std::endl;
    std::cout<<"average_value "<<average_value<<std::endl;
  }
}

void mdp_planner::Convert_to_Markov_chain(MDP_Net::Ptr& pNet)
{
  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    mdp_state_t *s = pNet->mdp_states[i];
    getTransitionModel(s,s->optimal_action,pNet);
    /*
    vector<double> probs;
    mdp_state_t *s = pNet->mdp_states[i];
    for(uint l=NORTH;l<NUM_ACTIONS;l++)
    {
      action_t act_curr = (action_t)l;
      mdp_planner::getTransitionModel(s,act_curr,pNet);
      double prob_curr = s->policy[l]*s->probs[l];
      probs.push_back(prob_curr);
    }
    for(int j=0;j<probs.size();j++)
    {
      s->probs[j]=probs[j];
    }
    */
  }
}

double mdp_planner::addSuccessorValue(mdp_state_t* s,action_t succ_a, MDP_Net::Ptr& pNet)
{
      mdp_planner::getTransitionModel(s,succ_a, pNet);
      //double obst_penalty   = -pParams->getParamNode()["mdp_methods"]["obst_penalty"].as<double>();
      //action_cost    = pParams->getParamNode()["mdp_methods"]["action_cost"].as<double>();
      double value=0;
      for(uint i=NORTH; i<NUM_ACTIONS; i++){
        /*
          if(s->successors[i] != s){

              mdp_state_t* next_st = (s->successors[action_t(i)]);

              double imm_rew = (next_st->type == GOAL) ? 100 : -1;
              value += s->probs[i] * (imm_rew + 0.9* next_st->optimal_value);

          }
          else
              value += s->probs[i] * -100;

              */
            mdp_state_t* next_st = (s->successors[action_t(i)]);

              //double imm_rew = (next_st->type == GOAL) ? 100 : -1;

              if(s->successors[i] !=s)
              {
                if(next_st->type==GOAL)
                {
                  double imm_rew = 100 + next_st->reachability;
                  value += s->probs[i] * (imm_rew + 0.9* next_st->last_optimal_value);
                }
                else if(next_st->type==OBSTACLE)
                {
                  double imm_rew = -100 + next_st->reachability;
                  value += s->probs[i] * (imm_rew + 0.9* s->last_optimal_value);
                }
                else
                {
                  double imm_rew = -1 + next_st->reachability;
                value += s->probs[i] * (imm_rew + 0.9* next_st->last_optimal_value);
                }
              }
              else
              {
                double imm_rew = -100 + next_st->reachability;
                value += s->probs[i] * (imm_rew + 0.9* s->last_optimal_value);
                //std::cout<<"IM here at successor is itself "<<s->probs[i]<<std::endl;
              }



      }
      //if obstacles, or walls such that robot cannot move forward but return to current state, put a penalty on it
      return value;
}

void mdp_planner::updateState(mdp_state_t* s, utils::Parameters::Ptr& pParams,MDP_Net::Ptr& pNet)
{
  if(s->type == BODY || s->type == START){
      //std::fill(s->q_values.begin(), s->q_values.end(), 0);
      //double max_val =  -INF; //s->optimal_value;
      //s->q_values[ZERO] = -INF;
      double max_val = -10000000;
      action_t optimal_act;
      for(uint j=NORTH; j<NUM_ACTIONS; j++)
      {
          action_t act = (action_t)j;
          if(s->successors[act] != s) //the act should be legal
          {
              getTransitionModel(s, act,pNet);
              //double direct_action_value = getQvalue(s, act);
              double direct_action_value = mdp_planner::addSuccessorValue(s,act,pNet);
              if (direct_action_value > max_val)
              {
                max_val = direct_action_value;
                s->optimal_action = (action_t)j;
              }
          }
      }
      //fill the max value as current value
      s->optimal_value = max_val;
      getTransitionModel(s,s->optimal_action,pNet);
}
}

mdp_state_t* mdp_planner::one_time_step_sample(mdp_state_t* s, action_t act,utils::Parameters::Ptr& pParams, MDP_Net::Ptr& pNet)
{
  double random_num;
  random_num = rand()%100;
  mdp_planner::getTransitionModel(s, act,pNet);
  double percent = random_num/100.0;
  double probability = 0.0;
  mdp_state_t* sample_next_state;
  for(uint i=NORTH;i<NUM_ACTIONS;i++)
  {
    probability += s->probs[i];
    if(percent < probability || percent == probability)
    {
      action_t sample_direct = (action_t)i;
      sample_next_state = s->successors[sample_direct];
      return sample_next_state;
    }
  }
}

double mdp_planner::Trail_update_(mdp_state_t* s,utils::Parameters::Ptr& pParams, MDP_Net::Ptr& pNet)
{
  int max_iter_num = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<int>();
//  std::cout<<"Im in trail_update"<<std::endl;
  //std::cout<<"max_iter_num "<<max_iter_num<<std::endl;
  int iter_num = 0;
  mdp_state_t* iter_state = s;
  mdp_state_t* iter_state_current;
  double error_all = 0;
  double error_current;
  while(iter_state->type != GOAL && iter_num < max_iter_num)
  {
    //std::cout<<"Im in trail_while"<<std::endl;
    double previous_value = iter_state->optimal_value;
    mdp_planner::updateState(iter_state, pParams, pNet);
    double current_value = iter_state->optimal_value;
    error_current = std::abs(previous_value - current_value);
    if(error_current > error_all)
    {
      error_all = error_current;
    }
    action_t opt_act = iter_state->optimal_action;
    for(uint j=NORTH;j<NUM_ACTIONS;j++)
    {
      action_t act_curr = (action_t)j;
      iter_state->actions[act_curr]=false;
    }
    iter_state->actions[opt_act]=true;
    iter_state_current = mdp_planner::one_time_step_sample(iter_state,opt_act,pParams,pNet);
    if(iter_state_current->type == OBSTACLE)
    {
      continue;
    }
    iter_state = iter_state_current;
    iter_num++;
  }
  return error_all;
}

void mdp_planner::Trail_Based_RTDP(MDP_Net::Ptr& pNet,utils::Parameters::Ptr& pParams)
{
  double error = 0;
  double error_one_trail;
  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    if(pNet->mdp_states[i]->type != OBSTACLE)
    {
      error_one_trail = mdp_planner::Trail_update_(pNet->mdp_states[i],pParams,pNet);
      if(error<error_one_trail)
      {
        error = error_one_trail;
      }
    }
  }
  std::cout<<"error is"<<error<<std::endl;
  while(error > 0.5)
  {
    error = 0;
    //error = mdp_planner::Trail_update_(pNet->mdp_states[0],pParams,pNet);

    for(int i=0;i<pNet->mdp_states.size();i++)
    {
      if(pNet->mdp_states[i]->type != OBSTACLE)
      {
        error_one_trail = mdp_planner::Trail_update_(pNet->mdp_states[i],pParams,pNet);
        if(error<error_one_trail)
        {
          error = error_one_trail;
        }
      }
    }
    std::cout<<"error is"<<error<<std::endl;

  }
}

void mdp_planner::Compute_MFPT(MDP_Net::Ptr& pNet, Disturbance::Ptr& pDisturb,utils::Parameters::Ptr& pParams)
{
  vector<Transform2> tf2_starts_1, tf2_goals_1;
  vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
  //std::cout<<"start position"<<std::endl<<vec[0]<<"  "<<vec[1]<<"  "<<vec[2]<<std::endl;
  tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
  vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
  tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));
  SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

  //high_resolution_clock::time_point t1 = high_resolution_clock::now();
  //_prioritizedstates.clear();
  //optimalActionTransitionDistribution(_states);
  pSSP->initTransMatrix();
  std::cout<<"goals "<<pNet->getState(tf2_goals_1[0].translation)->id<<std::endl;

  //pSSP->MFPT(pNet, pNet->getState(tf2_goals_1[0].translation)->id);
  pSSP->MFPT_obstacle(pNet);
  std::cout << "reachabilities" << std::endl;
  ofstream my_file;
  my_file.open("reachability.txt");

  for (int i = pNet -> n_rows - 1; i >= 0; i--) {
      for (int j = 0; j < pNet -> n_cols; j++) {
          //std::cout << i * pGrid2d -> n_cols + j << " ";
          std::cout<< pSSP->reachability[i * pNet -> n_cols + j]  << " ";
          my_file<<pSSP->reachability[i * pNet -> n_cols + j]<<",";
          pNet->mdp_states[i * pNet -> n_cols + j]->reachability = pSSP->reachability[i * pNet -> n_cols + j]*10-5;
      }
      std::cout << std::endl;
  }
  my_file.close();


  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    if(pNet->mdp_states[i]->type != OBSTACLE)
    {
    float max_val = INF;
    mdp_state_t* s = pNet->mdp_states[i];
    action_t optimal_act;
    for(uint j=NORTH; j<NUM_ACTIONS; j++)
    {
        action_t act = (action_t)j;
        if(s->successors[act] != s) //the act should be legal
        {
            getTransitionModel(s, act, pNet);
            //double direct_action_value = getQvalue(s, act);
            double direct_action_value = s->successors[act]->reachability;//mdp_planner::addSuccessorValue(s,act,pParams,pNet);
            if (direct_action_value < max_val)
            {
              max_val = direct_action_value;
              //s->optimal_action = (action_t)j;
              optimal_act = (action_t)j;
            }
        }
    }
    for(uint k=NORTH; k<NUM_ACTIONS; k++){
      s->policy[k]=0;
      //std::cout<<pNet->mdp_states[i]->probs[j]<<std::endl;
    }
    //std::cout<<"best_act "<<best_act<<std::endl;
    s->policy[optimal_act]=1.0;
    for(uint k=NORTH;k<NUM_ACTIONS;k++)
    {
      action_t act_curr = (action_t)k;
      s->actions[act_curr]=false;
    }
    s->actions[optimal_act]=true;
  }
  }
}

void mdp_planner::MFPT_RTDP(MDP_Net::Ptr& pNet,utils::Parameters::Ptr& pParams,Disturbance::Ptr& pDisturb)
{
  std::cout<<"Im here"<<std::endl;
  double error = 0;
  double error_one_trail;

  vector<Transform2> tf2_starts_1, tf2_goals_1;
  vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
  std::cout<<"start position"<<std::endl<<vec[0]<<"  "<<vec[1]<<"  "<<vec[2]<<std::endl;
  tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
  vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
  tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));
  mdp_planner::uniform_transition_initialization(pNet);
  //mdp_planner::Policy_iteration(pNet);
  //Convert_to_Markov_chain(pNet);
  //mdp_planner::Policy_evaluation(pNet);
  //mdp_planner::Policy_update(pNet);
  double converged_value = 0;
/*
  for(int j=0;j<10;j++){
  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    if(pNet->mdp_states[i]->type != OBSTACLE)
    {
      error_one_trail = mdp_planner::Trail_update_(pNet->mdp_states[i],pParams,pNet);
      if(error<error_one_trail)
      {
        error = error_one_trail;
      }
    }
    //std::cout<<"error is"<<error<<std::endl;
  }
  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    converged_value = converged_value + pNet->mdp_states[i]->optimal_value;
  }
  converged_value = converged_value/pNet->mdp_states.size();
  std::cout<<"converged_value is "<<converged_value<<std::endl;
  converged_value = 0;
}
*/
std::sort(pNet->mdp_obstacle_ids.begin(),pNet->mdp_obstacle_ids.end());
//for(int i=0;i<pNet->mdp_obstacle_ids.size();i++)
//{
//  std::cout<<"Obstacle_id "<<pNet->mdp_obstacle_ids[i]<<std::endl;
//}
Compute_MFPT(pNet, pDisturb, pParams);

/*
  SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

  //high_resolution_clock::time_point t1 = high_resolution_clock::now();
  //_prioritizedstates.clear();
  //optimalActionTransitionDistribution(_states);
  pSSP->initTransMatrix();
  std::cout<<"goals "<<pNet->getState(tf2_goals_1[0].translation)->id<<std::endl;
  pSSP->MFPT(pNet, pNet->getState(tf2_goals_1[0].translation)->id);
  std::cout << "reachabilities" << std::endl;
  ofstream my_file;
  my_file.open("reachability.txt");

  for (int i = pNet -> n_rows - 1; i >= 0; i--) {
      for (int j = 0; j < pNet -> n_cols; j++) {
          //std::cout << i * pGrid2d -> n_cols + j << " ";
          std::cout<< (int)pSSP->reachability[i * pNet -> n_cols + j]  << " ";
          my_file<<pSSP->reachability[i * pNet -> n_cols + j]<<",";
          pNet->mdp_states[i * pNet -> n_cols + j]->reachability = pSSP->reachability[i * pNet -> n_cols + j];
      }
      std::cout << std::endl;
  }
  my_file.close();


  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    if(pNet->mdp_states[i]->type != OBSTACLE)
    {
    float max_val = INF;
    mdp_state_t* s = pNet->mdp_states[i];
    action_t optimal_act;
    for(uint j=NORTH; j<NUM_ACTIONS; j++)
    {
        action_t act = (action_t)j;
        if(s->successors[act] != s) //the act should be legal
        {
            getTransitionModel(s, act, pNet);
            //double direct_action_value = getQvalue(s, act);
            double direct_action_value = s->successors[act]->reachability;//mdp_planner::addSuccessorValue(s,act,pParams,pNet);
            if (direct_action_value < max_val)
            {
              max_val = direct_action_value;
              s->optimal_action = (action_t)j;
              optimal_act = (action_t)j;
            }
        }
        for(uint k=NORTH; k<NUM_ACTIONS; k++){
          s->policy[k]=0;
          //std::cout<<pNet->mdp_states[i]->probs[j]<<std::endl;
        }
        //std::cout<<"best_act "<<best_act<<std::endl;
        s->policy[s->optimal_action]=1.0;
    }
    for(uint k=NORTH;k<NUM_ACTIONS;k++)
    {
      action_t act_curr = (action_t)k;
      s->actions[act_curr]=false;
    }
    s->actions[optimal_act]=true;
  }
  }
 */
  //mdp_planner::Policy_iteration(pNet);

  int num_iteration=0;
  mdp_planner::Policy_evaluation(pNet);
  mdp_planner::Policy_update(pNet);
  //Compute_MFPT(pNet,  pDisturb,pParams);
  double average_value=0;
  while(mdp_planner::check_policy_converge_PI(pNet->mdp_states))
  //for(int k=0;k<15;k++)
  {
    average_value = 0;
    mdp_planner::Policy_evaluation(pNet);
    mdp_planner::Policy_update(pNet);
    //Compute_MFPT(pNet,  pDisturb, pParams);
    for(int i = 0;i<pNet->mdp_states.size();i++)
    {
      average_value+=pNet->mdp_states[i]->optimal_value;
    }
    average_value = average_value/pNet->mdp_states.size();
    num_iteration++;
    std::cout<<"iternumber "<<num_iteration<<std::endl;
    std::cout<<"average_value "<<average_value<<std::endl;
  }

  //mdp_planner::Policy_evaluation(pNet);
  //mdp_planner::Policy_update(pNet);


/*
for(int i =0;i<pNet->n_rows*pNet->n_cols;i++)
{
  for(int j=0;j<pNet->n_rows*pNet->n_cols;j++)
  {
    std::cout<<pSSP->transMatrix.coeffRef(i,j)<<" ";
  }
  std::cout<<std::endl;
}
*/
}
