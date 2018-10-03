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


bool mdp_planner::check_policy_converge(std::vector<Policy_Node*> new_queue)
{
  for(int i=0;i<new_queue.size();i++)
  {
    if(new_queue[i]->optimal_act != new_queue[i]->last_optimal_act)
    {
      return true;
    }
  }
  return false;
}

float mdp_planner::compute_Q_value_PI(mdp_state_t* s,action_t act)
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
          state_value = state_value + 0.8*0.9*old_queue[i]->state_value;
        }
        else
        {
          state_value = state_value;
        }
    }
  }
  return state_value;
}

void mdp_planner::Policy_iteration(vector<mdp_state_t*>& states)
{
  srand((unsigned)time(0));
  for(vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
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
    s->last_optimal_acttion = random_act;
  }
  float error = 10;
  while(error<0.1)
  {
    error = 0
    for(vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
    {
        mdp_state_t *s = *itr;
        float new_value;
        new_value = mdp_planner::compute_Q_value_PI(s,s->optimal_action);
        float error_temp = std::abs(new_value-s->optimal_value);
        s->optimal_value = new_value;
        if(error_temp>error)
        {
          error = error_temp;
        }
    }
  }
  for(vector<mdp_state_t*>::iterator itr=states.begin();itr!=states.end();itr++)
  {

  }



}
