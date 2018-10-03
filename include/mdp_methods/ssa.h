
#ifndef SSA_H
#define SSA_H

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
#include "geometry_utils/Vector2.h"
#include <assert.h>
#include <iterator>
#include <algorithm>
#include "geometry_utils/Transform2.h"
#include "spline/UnconstrainedSplineFit.h"
#include "utils/parameters.h"
#include "utils/functions.h"
#include "disturbance/disturbance.h"
#include "mdp_net.h"
#include "mdp_core.h"
#include "mdp_state.h"

using namespace geometry_utils;

namespace mdp_planner
{
    struct CPT : public MDP_State
    {
      action_t policy_action;
      std::vector<mdp_state_t*> pre_state_variables;
      std::vector<float> trans_probabilities;

      CPT(mdp_state_t* s, action_t act, const MDP_Net::Ptr& pNet)
      {
        geometry_utils::Vec2 v_app = mdp_state_t::getActionVector(act);
        policy_action = act;
        for(int i=0;i<predecessors.size();i++)
        {
          if(s->predecessors[i] !=s)
          {
            Vec2 v_pre = pNet->cell_centers[s->id]-pNet->cell_centers[s->predecessors[i]->id];
            float cos_theta = v_app.dot(v_pre)/(v_app.norm()*v_pre.norm());
            if(cos_theta > 0.6)
            {
              pre_state_variables.push_back(s->predecessors[i]);
              trans_probabilities.push_back(0.9);
            }
            else
            {
              pre_state_variables.push_back(s->predecessors[i]);
              trans_probabilities.push_back(0.0);
            }
          }
        }
      }
    };
    struct Value_tree_node
    {
      float state_value;
      Value_tree_node* left;
      Value_tree_node* right;
      Value_tree_node* parent;


    };
    struct Q_action_tree_node
    {
      action_t act;
      float Q_action_value;
      Q_action_tree_node *left;
      Q_action_tree_node *right;
      Q_action_tree_node *parent;
    };
    struct Policy_Node : public MDP_State
    {
      std::vector<Policy_Node*> children;
      Policy_Node *parent;
      std::vector<Policy_Node*> self_Node;
      std::vector<action_t> acts;
      float state_value;
      Q_action_tree_node* Q_a_value;
      action_t optimal_act;
      mdp_state_t *regress_node;
      Policy_Node(mdp_state_t *s, action_t act, Policy_Node *Par)
      {
        regress_node = s;
        state_value = 0.0;
        optimal_act = act;
        parent = Par;
      }
    };
    float compute_Q_value(Policy_Node* Node,Policy_Node* root, const MDP_Net::Ptr& pNet);
    //float compute_Q_value(Policy_Node* Node,Policy_Node* root, const MDP_Net::Ptr& pNet);
    void SSA(Policy_Node* root,std::vector<Policy_Node*> queue_node, const MDP_Net::Ptr& pNet);
    //void SSA(std::vector<Policy_Node*> old_queue,std::vector<Policy_Node*> queue_node, const MDP_Net::Ptr& pNet);
    void update_policy(Policy_Node* root, std::vector<Policy_Node*> queue_node, const MDP_Net::Ptr& pNet);
    void GMSPI(mdp_state_t* goal, const MDP_Net::Ptr& pNet);
    bool find_index(std::vector<int> index, int k);
    bool check_predecessor(Policy_Node* queue,Policy_Node* node);
}

#endif
