
#ifndef METHOD_MANAGER_H
#define METHOD_MANAGER_H

#include "utils/data_loader.h"
#include "utils/parameters.h"
#include "utils/clock.h"
#include "vehicle/vehicle_model.h"
#include "mdp_methods/mdp_core.h"
#include "mdp_methods/ssp.h"
#include "mdp_methods/ssa.h"
#include "disturbance/disturbance.h"
#include "info_methods/info_core.h"
#include "geometry_utils/Pose.h"
#include <algorithm>

using namespace mdp_planner;
using namespace info_planner;


class MethodManager
{

public:

  typedef boost::shared_ptr<MethodManager> Ptr;
  typedef boost::shared_ptr<const MethodManager> ConstPtr;
  double *states_fpt_final;

  MethodManager(void);
  virtual ~MethodManager(void);

  //accessors
  utils::Parameters::Ptr getpParams(void){ return pParams; }
  utils::DataLoader::Ptr getpData(void){ return pData; }
  MDP_Grid2D::Ptr	getpGrid2d(void){ return pGrid2d; }
  MDP_Net::Ptr		getpNet(void){ return pNet; }
  Disturbance::Ptr 	getpDisturb(void){ return pDisturb; }
  MDP::Ptr 		getpMDP(void){ return pMDP; }
  Info::Ptr             getpInfo(void){ return pInfo; }
  SSP::Ptr              getpSSP(void){ return pSSP; }
  vector<AUVmodel::Ptr>	getpRobots(void){ return pRobots; }
  Vec3 			getBoundsXYZ(void){ return bounds_xyz; }
  int 			getNumRows(void){ return num_rows; }
  int 			getNumCols(void){ return num_cols; }

  void loadParams(void);

  void closeDoor(int);

  void openDoor(int);

  void removeGoal(int);

  void updatableStatesStart();

  //methods manager based on config yaml
  void methodManager(void);

  //basic/standard MDP method
  void mdpCore(void);

  //method applying imported vector field such as ROMS data
  void mdpImportedVecField(void);

  //method for information theoretical planning
  void infoPlan(void);

  //method for expected stochastic shortest path (SSP)
  void mdpExpectedSSP(void);

  vector<Transform2> tf2_starts, tf2_goals;

private:

  utils::Parameters::Ptr	pParams;
  utils::DataLoader::Ptr	pData;
  //MDP_Grid2D::Ptr		pGrid2d;
  MDP_Grid2D::Ptr   pGrid2d;
  //MDP_Grid2D::Ptr   pGrid2d_buffer;
  MDP_Grid2D::Ptr   pGrid2d_buff;
  //MDP_Net::Ptr			pNet;
  MDP_Net::Ptr      pNet;
  MDP_Net::Ptr      pNet_buff;
  Disturbance::Ptr 		pDisturb;
  Disturbance::Ptr    pDisturb_buff;
  MDP::Ptr	 		pMDP;
  MDP::Ptr      pMDP_buff;
  Info::Ptr			pInfo;
  SSP::Ptr			pSSP;
  vector<AUVmodel::Ptr> 	pRobots;

private:

  Vec3 bounds_xyz;
  int num_rows, num_cols;			//grid map dimension


};


#endif
