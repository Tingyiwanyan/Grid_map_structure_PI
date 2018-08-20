
#ifndef MDP_STATE_H 
#define MDP_STATE_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <cstddef>
#include <vector>
#include <set>
#include <map>
#include <limits>

#include <boost/shared_ptr.hpp>
#include "geometry_utils/Pose.h"

#ifndef INF
#define INF std::numeric_limits<int>::max()
#endif 

#ifndef EPSILON
#define EPSILON std::numeric_limits<float>::min()
#endif

using namespace std;


typedef unsigned int uint;
typedef uint ID;

const uint NUM_ACTIONS = 9;
typedef enum {ZERO=0, NORTH, NE, EAST, SE, SOUTH, SW, WEST, NW} action_t;
//typedef enum {ZERO=0, NORTH, EAST, SOUTH, WEST} action_t;
typedef enum {BODY, START, GOAL, OBSTACLE, REMOVED} type_t; 
	//REMOVED: finished goals //OBSTACLE may be moved out to ocp_states


namespace mdp_planner{

  typedef 
  struct MDP_State {

    typedef boost::shared_ptr<MDP_State> Ptr;
    typedef boost::shared_ptr<const MDP_State> ConstPtr;

    MDP_State(){ init_state(); }
    ~MDP_State(){}

    //point_t pos;		// position info should be obtained from grid2d

    int id;			// used this idx to retrieve cell centers 
    type_t type;		// such as an obstacle state
    bool color;			// whether visted or not

    vector<bool> actions; 		// NUM_ACTION actions, true means an action in selected direction 
    vector<MDP_State*> successors;	// default null, can be itself
    set<MDP_State*> predecessors;	// default null, can be itself
    vector<double> probs;		// transition probabilities, default 0
    vector<double> post_probs;
    vector<int> action_count;

    // value iterations
    vector<double> q_values;		// size=#actions, record q value for each action
    double optimal_value;		// the best q value
    action_t optimal_action;		// the best action 
    double last_optimal_value;		// standard VI are computed from values of last stage, a snapshot
    action_t last_optimal_action;

    MDP_State* spawn_parent; 	// spawn a copy of state by adding special properties, eg, goal/start
    map<MDP_State*, pair<double, action_t> > m_values;	//pair<value, optimal_action>, used in ssp
  

    /**** some light weight inline functions ****/

    //init state values
    inline void init_state(void){
      color = false;
      spawn_parent = NULL;
      actions.resize(NUM_ACTIONS, false);
      successors.resize(NUM_ACTIONS, this);
      predecessors.clear();
      probs.resize(NUM_ACTIONS, 0);
      post_probs.resize(NUM_ACTIONS, 0);
      optimal_value = 0;
      last_optimal_value = 0;
      q_values.resize(NUM_ACTIONS, 0);
      optimal_action = ZERO;
      last_optimal_action = ZERO;
      action_count.resize(NUM_ACTIONS, 0);
    }


    //reset state values
    inline void reset_state(void){
      optimal_value = 0; 
      optimal_action = ZERO;
      last_optimal_value=0;
      std::fill(actions.begin(), actions.end(), false);
      fill(q_values.begin(), q_values.end(), 0);
    }


    // return a unit vector specifying direction (assume origin is bottom-left)
    // if ZERO, then 0 vector returned. use only x-y plane here, z=0
    static geometry_utils::Vec2 getActionVector(action_t in) {
      // on horizontal plane, z=0;
      switch(in){
	case NORTH:
	  return geometry_utils::Vec2(0, 1); break;
	case NE:
	  return geometry_utils::Vec2(0.5*sqrt(2), 0.5*sqrt(2)); break;
	case EAST:
	  return geometry_utils::Vec2(1, 0); break;
	case SE:
	  return geometry_utils::Vec2(0.5*sqrt(2), -0.5*sqrt(2)); break;
	case SOUTH:
	  return geometry_utils::Vec2(0, -1); break;
	case SW:
	  return geometry_utils::Vec2(-0.5*sqrt(2), -0.5*sqrt(2)); break;
	case WEST:
	  return geometry_utils::Vec2(-1, 0); break;
	case NW:
	  return geometry_utils::Vec2(-0.5*sqrt(2), 0.5*sqrt(2)); break;
        case ZERO:
	  return geometry_utils::Vec2(0, 0); break;
	default:
	  cerr<<"failed returned a vector!"<<endl;
	  assert(0);
      }
    }

    //get the unit vector corresponding optimal action
    inline geometry_utils::Vec2 getOptimalActionVector(void) const{
      /*int mini = -2, maxi = 2;
      int output = mini + (rand() % (int)(maxi - mini + 1));
      int final_output = optimal_action + output;
      if(final_output >= NUM_ACTIONS) final_output = final_output % (NUM_ACTIONS - 1);
      if(final_output <= 0) final_output = NUM_ACTIONS - 1 + final_output;
      cout << optimal_action << ", " << output << ", " << final_output << endl;
      return  getActionVector((action_t)final_output);*/
      return  getActionVector(optimal_action);
    }


  } mdp_state_t;


}//namespace 

/*
typedef struct _Mstate {

  map<ID, State> states;

} Mstate;
*/

#endif


