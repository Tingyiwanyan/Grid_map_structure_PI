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
#include<queue>
#include<ctime>
#include<cstdlib>
#include<math.h>
#include<vector>
#include<iostream>

using namespace geometry_utils;
using namespace mdp_planner;

bool mdp_planner::check_predecessor(Policy_Node* queue,Policy_Node* node)
{
  for(int i=0;i<node->regress_node->predecessors.size();i++)
  {
    if(queue->regress_node->id==node->regress_node->predecessors[i]->id)
    {
      return true;
    }
  }
  return false;
}

float mdp_planner::compute_Q_value(Policy_Node* Node, action_t act, std::vector<Policy_Node*> old_queue, const MDP_Net::Ptr& pNet)
{
  //std::cout<<"Im here at mdp_planner"<<std::endl;
  //std::vector<Policy_Node*> queue;
  Policy_Node *temp;
  float state_value = 0;
  Vec2 v_app = mdp_state_t::getActionVector(act);
  if(Node->parent==NULL)
  {
    return 100;
  }
  else
  {/*
    //temp = Node->parent;
    queue.push_back(root);
    //std::cout<<"queue_size is "<<queue.size()<<std::endl;
    while(queue.size()!=0)
    {
      for(int i=0; i<queue[0]->children.size();i++)
      {
        //std::cout<<"child_size "<<queue[0]->children.size()<<std::endl;
        queue.push_back(queue[0]->children[i]);
      }
      if(queue[0]->regress_node->id !=Node->regress_node->id && check_predecessor(queue[0],Node))
      {
        Vec2 v_pre = pNet->cell_centers[queue[0]->regress_node->id]-pNet->cell_centers[Node->regress_node->id];
        float cos_theta = v_app.dot(v_pre)/(v_app.norm()*v_pre.norm());
        if(queue[0]->regress_node->id != Node->regress_node->id)
        {
          if(cos_theta > 0.9)
          {
            state_value = state_value + 0.8*0.9*queue[0]->state_value;
          }
          else
          {
            state_value = state_value;
          }
        }
      }
      //std::cout<<"state_value "<<state_value<<std::endl;
        queue.erase(queue.begin());
        //std::cout<<"queue_size2 is "<<queue.size()<<std::endl;
    }
    */
    //state_value = state_value - 1;

    for(int i=0;i<old_queue.size();i++)
    {
      if(old_queue[i]->regress_node->id !=Node->regress_node->id && check_predecessor(old_queue[i],Node))
      {
        Vec2 v_pre = pNet->cell_centers[old_queue[i]->regress_node->id]-pNet->cell_centers[Node->regress_node->id];
        float cos_theta = v_app.dot(v_pre)/(v_app.norm()*v_pre.norm());
        if(old_queue[i]->regress_node->id != Node->regress_node->id)
        {
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
  }
    //std::cout<<"value is"<<state_value<<std::endl;
    return state_value;
  }
}

void mdp_planner::update_policy(std::vector<Policy_Node*> old_queue, std::vector<Policy_Node*> queue_node,const MDP_Net::Ptr& pNet)
{
  for(int i=0; i<queue_node.size();i++)
  {
    action_t opt_act=queue_node[i]->optimal_act;
    float state_value = queue_node[i]->state_value;
    std::cout<<"old_state_value"<<state_value<<std::endl;
    for(int j=0;j<8;j++)
    {
      int k = j+1;
    //  queue_node[i]->optimal_act=(action_t)k;
      float state_value_temp = mdp_planner::compute_Q_value(queue_node[i],(action_t)k, old_queue,pNet);
      std::cout<<"value_temp "<<state_value_temp<<std::endl;
      if(state_value_temp>state_value)
      {
        opt_act = (action_t)k;
        state_value=state_value_temp;
      }
    }

    queue_node[i]->optimal_act=opt_act;
    //queue_node[i]->state_value=state_value;
    //queue_node[i]->regress_node->actions[opt_act]=true;
  }
}

void mdp_planner::SSA(std::vector<Policy_Node*> old_queue, std::vector<Policy_Node*> queue_node, const MDP_Net::Ptr& pNet)
{
  //std::cout<<"Im here at SSA"<<std::endl;
  float error=10;
  //Policy_Node* traverse=root;
  while(error>0.1)
  {
    error = 0;
    //traverse = root;
    //std::cout<<"queue_node_size is "<<queue_node.size()<<std::endl;
    for(int i=0;i<queue_node.size();i++)
    {
      float state_value = mdp_planner::compute_Q_value(queue_node[i],queue_node[i]->optimal_act, old_queue, pNet);
      float error_temp = std::abs(state_value-queue_node[i]->state_value);
      queue_node[i]->state_value = state_value;
      queue_node[i]->last_state_value = state_value;
      if(error_temp>error)
      {
        error = error_temp;
      }
    }
  //  std::cout<<"error is"<<error<<std::endl;
  /*
    while(traverse->right!=NULL)
    {
      float state_value = mdp_planner::compute_Q_value(traverse, pNet);
      float error_temp = state_value-traverse->state_value;
      traverse->state_value = state_value;
      if(error_temp>error)
      {
        error = error_temp;
      }
      traverse = traverse->right;
    }
  */
  }

}
bool mdp_planner::find_index(std::vector<int> index, int k)
{
  for(int i=0; i<index.size();i++)
  {
    if(index[i] == k)
    {
      return false;
    }
  }
  return true;
}

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

void mdp_planner::GMSPI(mdp_state_t* goal, const MDP_Net::Ptr& pNet)
{
  std::vector<action_t> act_temp;//used to compare policy
  std::vector<Policy_Node*> old_queue;//build for last time layer;
  std::vector<Policy_Node*> new_queue;//build for new layer for indexing;
  srand((unsigned)time(0));
  std::vector<int> index;
  mdp_planner::Policy_Node* root = new mdp_planner::Policy_Node(goal,ZERO,NULL);
  //root->parent = NULL;
  mdp_planner::Policy_Node* traverse = root;
  old_queue.push_back(root);
  index.push_back(goal->id);
  mdp_planner::SSA(old_queue,old_queue,pNet);
  int flag = 1;
  while(flag)
  {
    flag=0;
    for(int j=0;j<old_queue.size();j++)
    {
      for(int i=0;i<old_queue[j]->regress_node->predecessors.size();i++)
      {
        //std::cout<<"predecessors num is"<<old_queue[j]->regress_node->predecessors.size()<<std::endl;
        int ID = old_queue[j]->regress_node->predecessors[i]->id;
        //std::cout<<"ID "<<ID<<std::endl;
        if(mdp_planner::find_index(index,ID))
        {
          flag=1;
          int random_act_num = rand()%8;
          std::cout<<"act is"<<random_act_num<<std::endl;
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
          std::cout<<"random_act "<<random_act<<std::endl;
          mdp_planner::Policy_Node* new_node = new mdp_planner::Policy_Node(old_queue[j]->regress_node->predecessors[i], random_act,old_queue[j]);
          //old_queue[j]->regress_node->predecessors[i]->actions[random_act]=true;
          old_queue[j]->children.push_back(new_node);
          new_queue.push_back(new_node);
          index.push_back(ID);
          //std::cout<<"index size is "<<index.size()<<std::endl;
        }
      }
    }
  //  for(int n=0;n<new_queue.size();n++)
  //  {
    //  old_queue.push_back(new_queue[n]);
  //  }
    mdp_planner::SSA(old_queue,new_queue,pNet);
    mdp_planner::update_policy(old_queue, new_queue,pNet);
    //for(int i=0;i<5;i++)
    while(check_policy_converge(new_queue))
    {
      for(int d =0;d<new_queue.size();d++)
      {
        new_queue[d]->last_optimal_act=new_queue[d]->optimal_act;
        std::cout<<"optimal act"<<new_queue[d]->optimal_act<<std::endl;
      }
    mdp_planner::SSA(old_queue,new_queue,pNet);
    mdp_planner::update_policy(old_queue, new_queue,pNet);
    //mdp_planner::SSA(old_queue, new_queue,pNet);
    std::cout<<"Im here at optimal act"<<std::endl;
  }
  for(int i=0;i<new_queue.size();i++)
  {
    action_t opt_act = new_queue[i]->optimal_act;
    //new_queue[i]->optimal_act=new_queue[i]->last_optimal_act;
    new_queue[i]->regress_node->actions[opt_act]=true;
  }
    /*
    while(mdp_planner::check_policy_converge(new_queue))
    {
      for(int h=0;h<new_queue.size();h++)
      {
        new_queue[h]->last_act=new_queue[h]->optimal_act;
      }
      mdp_planner::SSA(old_queue,new_queue,pNet);
      mdp_planner::update_policy(old_queue, new_queue,pNet);
    }
    */
    old_queue.clear();
    for(int k=0;k<new_queue.size();k++)
    {
      old_queue.push_back(new_queue[k]);
    }
    new_queue.clear();
  }

}
