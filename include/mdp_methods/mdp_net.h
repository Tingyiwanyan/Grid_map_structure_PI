#ifndef MDP_NET_H
#define MDP_NET_H

#include "mdp_state.h" 
#include "mdp_grid2d.h" 
#include "../viz_tool/glfunc.h"

/*y default, 1st cell of grid start from bottom left; 
if from top left, comment below */

#define CELL_BEGIN_FROM_BOTTOM_LEFT

namespace mdp_planner{

  typedef 
  struct MDP_Net : public MDP_Grid2D
  {

    typedef boost::shared_ptr<MDP_Net> Ptr;
    typedef boost::shared_ptr<const MDP_Net> ConstPtr;

    std::vector<mdp_state_t*> mdp_states;
    std::vector<int> mdp_obstacle_ids;
    std::vector<int> reachablestates;
    std::vector<int> reachablestatesdg;
    std::vector<int> mdp_local_update_states;
    // associated with same idx as cell_centers
    
    viz_tool::RGB policy_color;
   
    MDP_Net(const MDP_Grid2D::Ptr& p)
    {
      initializeGrid2D(p->n_cols, p->n_rows, p->resolution, p->origin);
      cell_centers = p->cell_centers; //direct copy
      createStates();
      constructStatesTransitions();
      
      policy_color.r = 255;
      policy_color.g = 75;
      policy_color.b = 75;
    }
    MDP_Net(const MDP_Net::Ptr& p)
    {
      initializeGrid2D(p->n_cols, p->n_rows, p->resolution, p->origin);
      cell_centers = p->cell_centers;
      mdp_states.clear();
      for(uint i=0; i<p->mdp_states.size(); i++){
	mdp_state_t* s = new mdp_state_t(*p->mdp_states[i]);
        mdp_states.push_back(s);
      }
      constructStatesTransitions(); 
	 //it is important to construct new!!, as transition cannot be copied (copied the invalid ones)!
      
      policy_color.r = 255;
      policy_color.g = 75;
      policy_color.b = 75;
    }


    virtual ~MDP_Net()
    { 
      destroyStates(); 
      cout<<"\t mdp_net: destroyed all mdp states."<<endl;
    }


    //create states based on created grids
    virtual void createStates(void)
    {
      mdp_states.clear();
      if(cell_centers.empty())
        cerr<<"Warning: grid map is empty!!"<<endl;

      for(uint i=0; i<cell_centers.size(); i++){
	mdp_state_t* s = new mdp_state_t;
        s->id = i;
        //s->pos = p;
	s->type = BODY;
	mdp_states.push_back(s);
      }
    }


    virtual void destroyStates(void)
    {
      for(uint i=0; i<mdp_states.size(); i++)
        delete mdp_states[i];

      mdp_states.clear();
    }


    // given a point, and the cell w/h of grid, get corresponding state
    mdp_state_t* getState(const point2d_t& p, const point2d_t& _origin, double cell_width, double cell_height)
    {

      uint id = getID(p, _origin, cell_width, cell_height);
      return mdp_states[id];
    }

    mdp_state_t* getState(const point2d_t& p) 
    {  
       return getState(p, origin, resolution, resolution); 
    }

    
    //establish stochastic topology by linking adjacent states
    void constructStatesTransitions(void)
    {
      for(int i=0; i< (int)mdp_states.size(); i++){
	//TODO can save more time here
        //if(ocp_states[i]==OCCUPIED)
        //  continue;
        mdp_state_t* s = mdp_states[i];
        assert(s->id == i);

        int nc = n_cols;	//use int instead of unsigned int
       #ifdef CELL_BEGIN_FROM_BOTTOM_LEFT
        //north
        s->successors[NORTH] = (i + nc < mdp_states.size()) ? mdp_states[i + nc] : s;
        s->successors[NORTH]->predecessors.insert(s);
        //north east
        s->successors[NE] = ( (i + nc < mdp_states.size()) && (i%nc != nc - 1) ) ? mdp_states[i + nc + 1] : s;
        s->successors[NE]->predecessors.insert(s);
        //north west
        s->successors[NW] = ( (i + nc < mdp_states.size()) && (i%nc != 0) ) ? mdp_states[i + nc - 1] : s;
        s->successors[NW]->predecessors.insert(s);
        //south
        s->successors[SOUTH] = (i - nc >= 0) ? mdp_states[i - nc] : s;
        s->successors[SOUTH]->predecessors.insert(s);
        //south east
        s->successors[SE] = ( (i - nc >= 0) && (i%nc != nc - 1) ) ? mdp_states[i - nc + 1] : s;
        s->successors[SE]->predecessors.insert(s);
        //south west
        s->successors[SW] = ( (i - nc >= 0) && (i%nc != 0) ) ? mdp_states[i - nc - 1] : s;
        s->successors[SW]->predecessors.insert(s);
       #else
	//north
	s->successors[NORTH] = (i - nc >= 0) ? mdp_states[i - nc] : s;
	s->successors[NORTH]->predecessors.insert(s);
        //north east
        s->successors[NE] = ( (i - nc >= 0) && (i%nc != nc - 1) ) ? mdp_states[i - nc + 1] : s;
        s->successors[NE]->predecessors.insert(s);
        //north west
        s->successors[NW] = ( (i - nc >= 0) && (i%nc != 0) ) ? mdp_states[i - nc - 1] : s;
        s->successors[NW]->predecessors.insert(s);
	//south
	s->successors[SOUTH] = (i + nc < mdp_states.size()) ? mdp_states[i + nc] : s;
	s->successors[SOUTH]->predecessors.insert(s);
        //south east
        s->successors[SE] = ( (i + nc < mdp_states.size()) && (i%nc != nc - 1) ) ? mdp_states[i + nc + 1] : s;
        s->successors[SE]->predecessors.insert(s);
        //south west
        s->successors[SW] = ( (i + nc < mdp_states.size()) && (i%nc != 0) ) ? mdp_states[i + nc - 1] : s;
        s->successors[SW]->predecessors.insert(s);
       #endif

        //east
        s->successors[EAST] = (i%nc != nc - 1) ? mdp_states[i+1] : s;
        s->successors[EAST]->predecessors.insert(s);
        //west
        s->successors[WEST] = (i%nc != 0) ? mdp_states[i-1] : s;
        s->successors[WEST]->predecessors.insert(s);
      }//for i

    }


    //set obstacle states separately
    virtual void
    setObstacleStateValues(vector<mdp_state_t*>& obs, double val)
    {
      for(uint i=0; i<obs.size(); i++){
          obs[i]->optimal_value = val;
          obs[i]->last_optimal_value = val;
          //label it as obstacle
          obs[i]->type = OBSTACLE;
      }
    }


    //set obstacle states based on values in vector field matrix
    virtual void
    setObstacleStateValues(const vector<geometry_utils::Vec2>& vf_mat, double val)
    {
      //cout<<"vf size: "<<vf_mat.size()<<" states size: "<<mdp_states.size()<<endl;
      assert(vf_mat.size() == mdp_states.size());

      for(uint i=0; i<vf_mat.size(); i++)
          if(std::isnan(vf_mat[i].x()) || std::isnan(vf_mat[i].y()) ){
              mdp_states[i]->optimal_value = val;
              mdp_states[i]->last_optimal_value = val;
              mdp_states[i]->type = OBSTACLE;
          }
    }


    //bounding box version, corner1 and corner2 define diagnol of bounding box
    virtual void 
    setObstacleStateValues(uint bbx_corner1, uint bbx_corner2, double val)
    {
      if(n_cols == 0){
        cout<<"mdp_net: the grid n_cols needs to be non-zero!"<<endl;
        return;
      }
 
      int max_idx = (int)std::max(bbx_corner1, bbx_corner2); 
      int min_idx = (int)std::min(bbx_corner1, bbx_corner2); 

      int bbx_height = max_idx/n_cols - min_idx/n_cols + 1; 
      int bbx_width = std::abs(max_idx%(int)n_cols - min_idx%(int)n_cols) + 1;
      assert(fabs(bbx_width) <= n_cols); //double check no weird abs behavior

      //if max_idx is to the right of min_idx, +j, else -j
      int sign_j = (max_idx%n_cols > min_idx%n_cols) ? +1 : -1;
      for(int i=0; i<bbx_height; i++)
        for(int j=0; j<bbx_width; j++){
          int obs_idx = min_idx + i*n_cols + sign_j*j;
          //cout << obs_idx << " ";
          mdp_states[obs_idx]->optimal_value = val;
          mdp_states[obs_idx]->last_optimal_value = val;
          mdp_states[obs_idx]->type = OBSTACLE;
          mdp_states[obs_idx]->optimal_action = ZERO;
          //mdp_states[obs_idx]->last_optimal_action = ZERO;
          //mdp_states[obs_idx]->actions.resize(NUM_ACTIONS, false);
          std::fill(mdp_states[obs_idx]->actions.begin(), mdp_states[obs_idx]->actions.end(), false);
          std::fill(mdp_states[obs_idx]->q_values.begin(), mdp_states[obs_idx]->q_values.end(), 0);
          std::fill(mdp_states[obs_idx]->post_probs.begin(), mdp_states[obs_idx]->post_probs.end(), 0);

          //mdp_states[obs_idx]->probs[0] = 1;
          mdp_states[obs_idx]->post_probs[0] = 1;
          mdp_obstacle_ids.push_back(obs_idx);
        }
      cout << endl;
    }


    //bounding box version, corner1 and corner2 define diagnol of bounding box
    virtual void
    setNonObstacleStateValues(uint bbx_corner1, uint bbx_corner2)
    {
      if(n_cols == 0){
        cout<<"mdp_net: the grid n_cols needs to be non-zero!"<<endl;
        return;
      }

      int max_idx = (int)std::max(bbx_corner1, bbx_corner2);
      int min_idx = (int)std::min(bbx_corner1, bbx_corner2);

      int bbx_height = max_idx/n_cols - min_idx/n_cols + 1;
      int bbx_width = std::abs(max_idx%(int)n_cols - min_idx%(int)n_cols) + 1;
      assert(fabs(bbx_width) <= n_cols); //double check no weird abs behavior

      //if max_idx is to the right of min_idx, +j, else -j
      int sign_j = (max_idx%n_cols > min_idx%n_cols) ? +1 : -1;
      for(int i=0; i<bbx_height; i++)
        for(int j=0; j<bbx_width; j++){
          int obs_idx = min_idx + i*n_cols + sign_j*j;
          //cout << obs_idx << " ";
          mdp_states[obs_idx]->optimal_value = 0;
          mdp_states[obs_idx]->optimal_action = ZERO;
          mdp_states[obs_idx]->last_optimal_value = 0;
          mdp_states[obs_idx]->type = BODY;
          reachablestates.push_back(obs_idx);
          mdp_obstacle_ids.pop_back();
        }
      cout << endl;
    }

    virtual vector<int>
    statesWithinBoundingBox(uint bbx_corner1, uint bbx_corner2)
    {
      vector<int> stateSet;

      int max_idx = (int)std::max(bbx_corner1, bbx_corner2);
      int min_idx = (int)std::min(bbx_corner1, bbx_corner2);

      int bbx_height = max_idx/n_cols - min_idx/n_cols + 1;
      int bbx_width = std::abs(max_idx%(int)n_cols - min_idx%(int)n_cols) + 1;
      assert(fabs(bbx_width) <= n_cols); //double check no weird abs behavior

      //if max_idx is to the right of min_idx, +j, else -j
      int sign_j = (max_idx%n_cols > min_idx%n_cols) ? +1 : -1;
      for(int i=0; i<bbx_height; i++)
        for(int j=0; j<bbx_width; j++){
          int obs_idx = min_idx + i*n_cols + sign_j*j;
          //cout << obs_idx << " ";

          stateSet.push_back(obs_idx);
        }
      cout << endl;
      return stateSet;
    }


    virtual void
    setObstacleStateValues(double val){}
    
    void setPolicyColor(const viz_tool::RGB &color) {
      policy_color = color;
    }
    
    viz_tool::RGB getPolicyColor() {
      return policy_color;
    }


  } mdp_net_t;

}

#endif

