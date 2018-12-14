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
#include <fstream>
#include <queue>
#include<ctime>
#include<cstdlib>
#include<math.h>
#include<vector>
#include<iostream>

using namespace geometry_utils;
using namespace mdp_planner;


bool mdp_planner::check_policy_converge_PI(std::vector<mdp_state_t*>& states)
{
  for(std::vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
  {
    mdp_state_t *s = *itr;
    if(s->type == BODY || s->type == START){
    if(s->optimal_action != s->last_optimal_action)
    {
      return true;
    }
  }
  }
  return false;
}

float mdp_planner::compute_Q_value_PI(mdp_state_t* s,action_t act,const MDP_Net::Ptr& pNet)
{
  float state_value = 0;
  Vec2 v_app = mdp_state_t::getActionVector(act);
  if(s->type==GOAL)
  {
    return 100;
  }
  else
  {
    for(int i=0; i<s->successors.size();i++)
    {

        Vec2 v_pre = pNet->cell_centers[s->successors[i]->id]-pNet->cell_centers[s->id];
        float cos_theta = v_app.dot(v_pre)/(v_app.norm()*v_pre.norm());
        if(cos_theta > 0.9)
        {
          state_value = state_value + 0.8*0.9*s->successors[i]->optimal_value;
        }
        else
        {
          state_value = state_value;
        }
    }
  }
  return state_value;
}

void mdp_planner::Policy_iteration(std::vector<mdp_state_t*>& states,const MDP_Net::Ptr& pNet)
{
  srand((unsigned)time(0));
  for(std::vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
  {
    int random_act_num = rand()%8;
    //std::cout<<"act is"<<random_act_num<<std::endl;
    action_t random_act=ZERO;
    switch(random_act_num)
    {
      case 0:
      random_act = NORTH;
      break;
      case 1:
      random_act = NE;
      break;
      case 2:
      random_act = EAST;
      break;
      case 3:
      random_act = SE;
      break;
      case 4:
      random_act = SOUTH;
      break;
      case 5:
      random_act = SW;
      break;
      case 6:
      random_act = WEST;
      break;
      case 7:
      random_act = NW;
      break;
      default:
      exit(0);
    }
    mdp_state_t *s = *itr;
    s->optimal_action = random_act;
    s->last_optimal_action = random_act;
  }
  float error = 10;
  while(error>0.1)
  {
    error = 0;
    for(std::vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
    {
        mdp_state_t *s = *itr;
        float new_value;
        new_value = mdp_planner::compute_Q_value_PI(s,s->optimal_action,pNet);
        float error_temp = std::abs(new_value-s->optimal_value);
        s->optimal_value = new_value;
        if(error_temp>error)
        {
          error = error_temp;
        }
    }
  }
  for(std::vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
  {
    mdp_state_t *s = *itr;
    action_t opt_act=s->optimal_action;
    float state_value = s->optimal_value;
    std::cout<<"old_state_value"<<s->optimal_value<<std::endl;
    for(int j=0;j<8;j++)
    {
      int k = j+1;
    //  queue_node[i]->optimal_act=(action_t)k;
      float state_value_temp = mdp_planner::compute_Q_value_PI(s,(action_t)k,pNet);
      std::cout<<"value_temp "<<state_value_temp<<std::endl;
      if(state_value_temp>state_value)
      {
        opt_act = (action_t)k;
        state_value=state_value_temp;
      }
    }

    s->optimal_action=opt_act;
  }
 while(check_policy_converge_PI(states))
  {
    std::cout<<"Im here at policy update"<<std::endl;
    for(std::vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
    {
        mdp_state_t *s = *itr;
        s->last_optimal_action=s->optimal_action;
    }
    float error = 10;
    while(error>0.1)
    {
      //std::cout<<"Im here in policy iteration"<<std::endl;
      error = 0;
      for(std::vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
      {
          mdp_state_t *s = *itr;
          float new_value;
          new_value = mdp_planner::compute_Q_value_PI(s,s->optimal_action,pNet);
          std::cout<<"new value "<<new_value<<std::endl;
          float error_temp = std::abs(new_value-s->optimal_value);
          s->optimal_value = new_value;
          if(error_temp>error)
          {
            error = error_temp;
          }
      }
    }
    for(std::vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
    {
      mdp_state_t *s = *itr;
      action_t opt_act=s->optimal_action;
      float state_value = s->optimal_value;
      std::cout<<"old_state_value"<<state_value<<std::endl;
      for(int j=0;j<8;j++)
      {
        int k = j+1;
      //  queue_node[i]->optimal_act=(action_t)k;
        float state_value_temp = mdp_planner::compute_Q_value_PI(s,(action_t)k,pNet);
        std::cout<<"value_temp "<<state_value_temp<<std::endl;
        if(state_value_temp>state_value)
        {
          opt_act = (action_t)k;
          state_value=state_value_temp;
        }
      }

      s->optimal_action=opt_act;
    }
  }
  for(std::vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
  {
      mdp_state_t *s = *itr;
      action_t opt_act = s->optimal_action;
      s->actions[opt_act]=true;
  }
}
