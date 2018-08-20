#ifndef DISTURBANCE_H
#define DISTURBANCE_H

#include "geometry_utils/Rotation2.h"
#include "geometry_utils/Pose.h"
#include "mdp_methods/mdp_grid2d.h"
#include "utils/data_loader.h"
#include "utils/clock.h"
#include "utils/parameters.h"

using namespace geometry_utils;


class Disturbance 
{

public:

  typedef boost::shared_ptr<Disturbance> Ptr;
  typedef boost::shared_ptr<const Disturbance> ConstPtr;
 
  Disturbance(const utils::Parameters::Ptr&, const mdp_planner::MDP_Grid2D::Ptr&);
  Disturbance(const utils::Parameters::Ptr&, const utils::DataLoader::Ptr&, const mdp_planner::MDP_Grid2D::Ptr&);
  Disturbance(const utils::Parameters::Ptr&, const vector<Vec2>&);
  virtual ~Disturbance();

  mdp_planner::MDP_Grid2D::Ptr 	getpGrid2d(void) { return pGrid2d; }
  std::vector<Vec2>& 		getVecField(void) { return vec_field; }
  std::vector<Vec2> 		getVecField(uint frame);
  Vec2 				getSwirlCenter(void) { return swirl_center; }
  Vec2 				getVec(uint id); 
  //Vec2 				getVec(const mdp_planner::mdp_state_t* s); 
  //Vec2 				getVec(const point2d_t& p); 

  void loadParams(void);

  //update vec field along time evolution
  void updateDynamicVecField(void);
  void updateDynamicVecFieldNC(void); //NC version

  //function includes a couple of time-varying methods
  Vec2 predictDisturbance(const point2d_t& pt, double t);

  //disturbance prediction of NC data
  Vec2 predictDisturbanceNC(const point2d_t& pt, double t);

  //tested, but not used, performance not good
  void meanderVecField(void);
  
private:

  utils::Parameters::Ptr 	pParams;
  utils::DataLoader::Ptr 	pData;
  mdp_planner::MDP_Grid2D::Ptr 	pGrid2d;
  std::vector<Vec2>		vec_field;	//currently only 2d vector field

  //auxiliary vars
  Vec2 v_uniform;
  Vec2 swirl_center;   //for swirl/centripetal center
  //bool dynamic_vf;

};



#endif


