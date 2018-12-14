#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <map>
#include <chrono>
#include <fstream>

#include "method_manager/method_manager.h"
#include "info_methods/dp_solver.h"
#include "info_methods/greedy_solver.h"
#include "viz_tool/glfunc.h"
#include "mdp_methods/ssa.h"

#include <boost/filesystem.hpp>
#include <vector>

extern std::string dir_name;
int highgoalindex = -1, r;

using namespace std::chrono;
using namespace mdp_planner;

MethodManager::MethodManager(void) {

  // read parameters
  string config_file("../configs/config.yaml");
  pParams = utils::Parameters::Ptr(new utils::Parameters(config_file));

  //load common params
  loadParams();

  //choose a method based on config
  methodManager();

}

MethodManager::~MethodManager() {
}

void
MethodManager::loadParams(void) {

  bounds_xyz.x() = pParams->getParamNode()["environment"]["coordinate_bounds"]["bound_x"].as<double>();
  bounds_xyz.y() = pParams->getParamNode()["environment"]["coordinate_bounds"]["bound_y"].as<double>();
  bounds_xyz.z() = pParams->getParamNode()["environment"]["coordinate_bounds"]["bound_z"].as<double>();

  int num_starts = pParams->getParamNode()["start_goal_config"]["num_starts"].as<int>();
  int num_goals = pParams->getParamNode()["start_goal_config"]["num_goals"].as<int>();

  for (uint i = 0; i < num_starts; i++) {
    std::ostringstream sstream;
    sstream << "s" << i;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"][sstream.str()].as<vector<double> >();
    cout << "start pose [" << i << "]: " << vec << endl;
    tf2_starts.push_back(Transform2(vec[0], vec[1], vec[2]));
  }
  for (uint i = 0; i < num_goals; i++) {
    std::ostringstream sstream;
    sstream << "g" << i;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_goals"][sstream.str()].as<vector<double> >();
    cout << "goal pose [" << i << "]: " << vec << endl;
    tf2_goals.push_back(Transform2(vec[0], vec[1], vec[2]));
  }

}

void
MethodManager::updatableStatesStart(){
    vector<int> startSet = pNet->statesWithinBoundingBox(330,693);
    for(vector<int>::iterator it = startSet.begin(); it != startSet.end(); ++it)
    {
        pNet->reachablestates.push_back(*it);
        pNet->reachablestatesdg.push_back(*it);
    }
}

void
MethodManager::closeDoor(int doornumber)
{
    uint index33, index34;

    //r = ((double) rand() / RAND_MAX) * 3;
    r = doornumber;
    std::cout << "\n\nClosing Door: " << r << std::endl;

    if(r == 0){
        index33 = pParams->getParamNode()["obstacles"]["bbx17a"].as<int>();
        index34 = pParams->getParamNode()["obstacles"]["bbx17b"].as<int>();
    }
    else if(r == 1){
        index33 = pParams->getParamNode()["obstacles"]["bbx18a"].as<int>();
        index34 = pParams->getParamNode()["obstacles"]["bbx18b"].as<int>();
    }
    else if(r == 2){
        index33 = pParams->getParamNode()["obstacles"]["bbx19a"].as<int>();
        index34 = pParams->getParamNode()["obstacles"]["bbx19b"].as<int>();
    }

    pNet->reachablestates.clear();
    //updatableStatesStart();
    pNet->setObstacleStateValues(index33, index34, pMDP->getObstPenalty());

    pSSP->initTransMatrix();
    double fpt_val = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts[0].translation), pNet->getState(tf2_goals[r].translation)->id);

    std::cout << "Unreachable states to High Goal: " << pNet->reachablestates.size() << std::endl;

    if(fpt_val > 500)
    {
        std::cout << "Need to update policy locally" << std::endl;

        uint num_iterations = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
        for (uint i = 0; i < num_iterations; i++)
        {
            cout << i << " " << std::flush;
            pMDP->valueIterationLocalStates(pNet->mdp_states,pNet->reachablestates,"forward");
        }
        //pMDP->optimalActionTransitionDistribution(pNet->mdp_states);

    }
    else{
        std::cout << "No need to update policy" << std::endl;
    }

}

void
MethodManager::openDoor(int doornumber)
{
    uint index33, index34;

    r = doornumber;
    std::cout << "\n\nOpening Door: " << r << std::endl;

    if(r == 0){
        index33 = pParams->getParamNode()["obstacles"]["bbx17a"].as<int>();
        index34 = pParams->getParamNode()["obstacles"]["bbx17b"].as<int>();
    }
    else if(r == 1){
        index33 = pParams->getParamNode()["obstacles"]["bbx18a"].as<int>();
        index34 = pParams->getParamNode()["obstacles"]["bbx18b"].as<int>();
    }
    else if(r == 2){
        index33 = pParams->getParamNode()["obstacles"]["bbx19a"].as<int>();
        index34 = pParams->getParamNode()["obstacles"]["bbx19b"].as<int>();
    }

    pNet->reachablestates.clear();
    pNet->setNonObstacleStateValues(index33, index34);

    if(r == highgoalindex)
    {
        std::cout << "Need to update policy globally" << std::endl;
        pMDP->iterations();
        pMDP->optimalActionTransitionDistribution(pNet->mdp_states);

    }
    else
    {
        pSSP->initTransMatrix();
        double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts[0].translation), pNet->getState(tf2_goals[r].translation)->id);

        std::cout << "Unreachable states to High Goal: " << pNet->reachablestates.size() << std::endl;

        if(fpt_val_n > 500)
        {
            std::cout << "Need to update policy locally" << std::endl;
            uint num_iterations = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
            for (uint i = 0; i < num_iterations; i++)
            {
                cout << i << " " << std::flush;
                pMDP->valueIterationLocalStates(pNet->mdp_states,pNet->reachablestates,"forward");
            }
            //pMDP->optimalActionTransitionDistribution(pNet->mdp_states);
        }
        else{
            std::cout << "No need to update policy" << std::endl;
        }
    }
}


void
MethodManager::removeGoal(int goal_id)
{

    std::cout << "\n\nRemoving Goal: " << goal_id << "\n" << std::endl;

    pNet->reachablestatesdg.clear();

    pNet->getState(tf2_goals[goal_id].translation)->type = BODY;
    pMDP->fillTypeValue(pNet->getState(tf2_goals[goal_id].translation), BODY, 0);
    pNet->reachablestatesdg.push_back(pNet->getState(tf2_goals[goal_id].translation)->id);
    updatableStatesStart();


    if(goal_id == highgoalindex)
    {



        std::cout << "Need to update policy globally" << std::endl;
        pMDP->iterations();
        pMDP->optimalActionTransitionDistribution(pNet->mdp_states);

    }
    else
    {
        std::cout << "Need to update policy locally" << std::endl;

        pMDP->optimalActionTransitionDistribution(pNet->mdp_states);
        pSSP->initTransMatrix();
        double fpt_v = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts[0].translation), pNet->getState(tf2_goals[goal_id].translation)->id);


        std::cout << "Unreachable states to Removed Goal: " << pNet->reachablestatesdg.size() << std::endl;

        uint num_iterations = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
        for (uint i = 0; i < num_iterations; i++)
        {
            cout << i << " " << std::flush;
            pMDP->valueIterationLocalStates(pNet->mdp_states,pNet->reachablestatesdg,"forward");
        }
    }
}

void
MethodManager::methodManager(void) {

  string method = pParams->getParamNode()["method_manager"].as<string>();

  if (method.compare("mdp_core") == 0) {
    cout << "MDP core methd is selected." << endl;
    mdpCore();
  } else if (method.compare("imported_vf") == 0) {
    cout << "Imported vector field disturbance is selected." << endl;
    mdpImportedVecField();
  } else if (method.compare("info_plan") == 0) {
    cout << "Information theoretical planning method is selected." << endl;
    infoPlan();
  } else if (method.compare("expected_ssp") == 0) {
    cout << "Expected stochastic shortest path method is selected." << endl;
    mdpExpectedSSP();
  }
  else {
    cout << "Method does not exist!" << endl;
    exit(0);
  }

}
/*
void
MethodManager::mdpCore(void) {

  num_rows = pParams->getParamNode()["environment"]["grids"]["num_rows"].as<unsigned int>();
  num_cols = pParams->getParamNode()["environment"]["grids"]["num_cols"].as<unsigned int>();

  int divide = 2;
  int divide_rows = num_rows/divide;
  int divide_cols = num_cols/divide;

  std::cout<<"divide_rows"<<divide_rows<<std::endl;
  std::cout<<"divide_cols"<<divide_cols<<std::endl;
  bool update = false;
  bool initial = true;

  point2d_t origin(-bounds_xyz.x(), -bounds_xyz.y());
  for(int ii = 0; ii < divide_cols; ii++)
  {
    std::cout<<"Im in"<<ii<<"th later"<<std::endl;
    double resolution = 2.0 * bounds_xyz.x() / ((ii+1)*divide);

    cout << "Grid map dimension: " << num_rows << "x" << num_cols << "; origin: " << origin << endl;
    if(initial == true)
    {
      pGrid2d = new MDP_Grid2D::Ptr(new MDP_Grid2D(divide, divide, resolution, origin));
      pNet = new MDP_Net::Ptr(new MDP_Net(&pGrid2d));
      pDisturb = new Disturbance::Ptr(new Disturbance(pParams, &pGrid2d));
      std::cout<<"cout grid map"<<std::endl<<pGrid2d->cell_centers[0]<<std::endl;
    }
    else{
      std::cout<<"Im here in assign pnet in update=true"<<std::endl;
      if(update == true)
      {
        pGrid2d_buffer = new MDP_Grid2D::Ptr(new MDP_Grid2D((ii+1)*divide, (ii+1)*divide, resolution, origin));

        pNet_buffer = new MDP_Net::Ptr(new MDP_Net(pGrid2d_buffer));
        pDisturb_buffer = new Disturbance::Ptr(new Disturbance(pParams, pGrid2d_buffer));
        std::cout<<"cout grid map"<<std::endl<<pGrid2d_buffer->cell_centers[0]<<std::endl;
        for(int i=0; i<&pGrid2d_buffer->cell_centers.size();i++)
        {
          //std::cout<<"IM here in first for"<<std::endl;
          int index = -1;
          float distance_optimal = INF;
          for(int j=0; j<&pGrid2d->cell_centers.size();j++)
          {
            //std::cout<<"Im here in second for"<<std::endl;
            float x_dist = &pGrid2d->cell_centers[j][0]-&pGrid2d_buffer->cell_centers[i][0];
            float y_dist = &pGrid2d->cell_centers[j][1]-&pGrid2d_buffer->cell_centers[j][1];
            float distance = std::sqrt(x_dist*x_dist+y_dist*y_dist);
            //std::cout<<"This is distance"<<distance<<std::endl;
            if(distance < distance_optimal)
            {
              distance_optimal = distance;
              index = j;
            }
          }
          //std::cout<<"This is index"<<index<<std::endl;
          //std::cout<<"Im here in assign optimal value"<<std::endl;
          //pNet_buffer->mdp_states[i]->optimal_value = pNet->mdp_states[index]->optimal_value;
          &pNet_buffer->mdp_states[i]->optimal_action = &pNet->mdp_states[index]->optimal_action;
        }
      }
      else
      {
        pGrid2d = new MDP_Grid2D::Ptr(new MDP_Grid2D((ii+1)*divide, (ii+1)*divide, resolution, origin));
        pNet = new MDP_Net::Ptr(new MDP_Net(&pGrid2d));
        pDisturb = new Disturbance::Ptr(new Disturbance(pParams, &pGrid2d));
        std::cout<<"cout grid map"<<std::endl<<&pGrid2d_buffer->cell_centers[0]<<std::endl;
        for(int i=0; i<&pGrid2d->cell_centers.size();i++)
        {
          //std::cout<<"IM here in first for"<<std::endl;
          int index = -1;
          float distance_optimal = INF;
          for(int j=0; j<&pGrid2d_buffer->cell_centers.size();j++)
          {
            //std::cout<<"Im here in second for"<<std::endl;
            float x_dist = &pGrid2d->cell_centers[j][0]-&pGrid2d_buffer->cell_centers[i][0];
            float y_dist = &pGrid2d->cell_centers[j][1]-&pGrid2d_buffer->cell_centers[j][1];
            float distance = std::sqrt(x_dist*x_dist+y_dist*y_dist);
            //std::cout<<"This is distance"<<distance<<std::endl;
            if(distance < distance_optimal)
            {std::cout << "\nmap indices" << std::endl;
    for (int i = &pGrid2d -> n_rows - 1; i >= 0; i--) {
        for (int j = 0; j < &pGrid2d -> n_cols; j++) {
            std::cout << i * &pGrid2d -> n_cols + j << " ";
        }
        std::cout << std::endl;
    }
              distance_optimal = distance;
              index = j;
            }
          }
          //std::cout<<"This is index"<<index<<std::endl;
          //std::cout<<"Im here in assign optimal value"<<std::endl;
          //pNet_buffer->mdp_states[i]->optimal_value = pNet->mdp_states[index]->optimal_value;
          &pNet->mdp_states[i]->optimal_action = &pNet_buffer->mdp_states[index]->optimal_action;
      }
    }
  }


    if(update == false)
    {
      if (pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0 && pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
        pMDP = new MDP::Ptr(new MDP(pParams, &pNet, &pDisturb));
    }
    }
    else
    {
      std::cout<<"Im here in assign pMDP in ture"<<std::endl;
      if (pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0 && pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
        pMDP_buffer = new MDP::Ptr(new MDP(pParams, &pNet_buffer, &pDisturb_buffer));
    }
    }

      // set the state type
    if(update == false)
    {
      for (uint i = 0; i < tf2_starts.size(); i++) {
        &pNet->getState(tf2_starts[i].translation)->type = START;
      }
      for (uint i = 0; i < tf2_goals.size(); i++) {
        double goal_value = MethodManager::PtrpParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
        &pNet->getState(tf2_goals[i].translation)->type = GOAL;
        if(i==1){
            highgoalindex = i;
            &pMDP->fillTypeValue(&pNet->getState(tf2_goals[i].translation), GOAL, goal_value + 50);
        }
        else{
            &pMDP->fillTypeValue(&pNet->getState(tf2_goals[i].translation), GOAL, goal_value);
        }
      }
    }
    else
    {
      for (uint i = 0; i < tf2_starts.size(); i++) {
        &pNet_buffer->getState(tf2_starts[i].translation)->type = START;
      }
      for (uint i = 0; i < tf2_goals.size(); i++) {
        double goal_value = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
        &pNet_buffer->getState(tf2_goals[i].translation)->type = GOAL;
        if(i==1){
            highgoalindex = i;
            &pMDP_buffer->fillTypeValue(&pNet_buffer->getState(tf2_goals[i].translation), GOAL, goal_value + 50);
        }
        else{
            &pMDP_buffer->fillTypeValue(&pNet_buffer->getState(tf2_goals[i].translation), GOAL, goal_value);
        }
      }
    }

      // Including obstacles
      bool hasObs = pParams->getParamNode()["obstacles"]["hasObs"].as<bool>();
      if (hasObs) {
          //cout << "I am here 2.5" << endl;
          uint index1 = pParams->getParamNode()["obstacles"]["bbx1a"].as<int>();
          uint index2 = pParams->getParamNode()["obstacles"]["bbx1b"].as<int>();
          uint index3 = pParams->getParamNode()["obstacles"]["bbx2a"].as<int>();
          uint index4 = pParams->getParamNode()["obstacles"]["bbx2b"].as<int>();
          uint index5 = pParams->getParamNode()["obstacles"]["bbx3a"].as<int>();
          uint index6 = pParams->getParamNode()["obstacles"]["bbx3b"].as<int>();
          uint index7 = pParams->getParamNode()["obstacles"]["bbx4a"].as<int>();
          uint index8 = pParams->getParamNode()["obstacles"]["bbx4b"].as<int>();
          uint index9 = pParams->getParamNode()["obstacles"]["bbx5a"].as<int>();
          uint index10 = pParams->getParamNode()["obstacles"]["bbx5b"].as<int>();
          uint index11 = pParams->getParamNode()["obstacles"]["bbx6a"].as<int>();
          uint index12 = pParams->getParamNode()["obstacles"]["bbx6b"].as<int>();
          uint index13 = pParams->getParamNode()["obstacles"]["bbx7a"].as<int>();
          uint index14 = pParams->getParamNode()["obstacles"]["bbx7b"].as<int>();
          uint index15 = pParams->getParamNode()["obstacles"]["bbx8a"].as<int>();
          uint index16 = pParams->getParamNode()["obstacles"]["bbx8b"].as<int>();
          uint index17 = pParams->getParamNode()["obstacles"]["bbx9a"].as<int>();
          uint index18 = pParams->getParamNode()["obstacles"]["bbx9b"].as<int>();
          uint index19 = pParams->getParamNode()["obstacles"]["bbx10a"].as<int>();
          uint index20 = pParams->getParamNode()["obstacles"]["bbx10b"].as<int>();
          uint index21 = pParams->getParamNode()["obstacles"]["bbx11a"].as<int>();
          uint index22 = pParams->getParamNode()["obstacles"]["bbx11b"].as<int>();
          uint index23 = pParams->getParamNode()["obstacles"]["bbx12a"].as<int>();
          uint index24 = pParams->getParamNode()["obstacles"]["bbx12b"].as<int>();
          uint index25 = pParams->getParamNode()["obstacles"]["bbx13a"].as<int>();
          uint index26 = pParams->getParamNode()["obstacles"]["bbx13b"].as<int>();
          uint index27 = pParams->getParamNode()["obstacles"]["bbx14a"].as<int>();
          uint index28 = pParams->getParamNode()["obstacles"]["bbx14b"].as<int>();
          uint index29 = pParams->getParamNode()["obstacles"]["bbx15a"].as<int>();
          uint index30 = pParams->getParamNode()["obstacles"]["bbx15b"].as<int>();
          uint index31 = pParams->getParamNode()["obstacles"]["bbx16a"].as<int>();
          uint index32 = pParams->getParamNode()["obstacles"]["bbx16b"].as<int>();

          uint index33 = pParams->getParamNode()["obstacles"]["bbx20a"].as<int>();
          uint index34 = pParams->getParamNode()["obstacles"]["bbx20b"].as<int>();
          uint index35 = pParams->getParamNode()["obstacles"]["bbx21a"].as<int>();
          uint index36 = pParams->getParamNode()["obstacles"]["bbx21b"].as<int>();
          uint index37 = pParams->getParamNode()["obstacles"]["bbx22a"].as<int>();
          uint index38 = pParams->getParamNode()["obstacles"]["bbx22b"].as<int>();
          uint index39 = pParams->getParamNode()["obstacles"]["bbx23a"].as<int>();
          uint index40 = pParams->getParamNode()["obstacles"]["bbx23b"].as<int>();
          uint index41 = pParams->getParamNode()["obstacles"]["bbx24a"].as<int>();
          uint index42 = pParams->getParamNode()["obstacles"]["bbx24b"].as<int>();
          */

          /*pNet->setObstacleStateValues(index1, index2, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index3, index4, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index5, index6, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index7, index8, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index9, index10, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index11, index12, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index13, index14, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index15, index16, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index17, index18, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index19, index20, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index21, index22, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index23, index24, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index25, index26, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index27, index28, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index29, index30, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index31, index32, pMDP->getObstPenalty());*/
/*
          pNet->setObstacleStateValues(index33, index34, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index35, index36, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index37, index38, pMDP->getObstPenalty());
          pNet->setObstacleStateValues(index39, index40, pMDP->getObstPenalty());
          //pNet->setObstacleStateValues(index41, index42, pMDP->getObstPenalty());
*/

          /*
          uint door1a = pParams->getParamNode()["obstacles"]["bbx17a"].as<int>();
          uint door1b = pParams->getParamNode()["obstacles"]["bbx17b"].as<int>();

          uint door2a = pParams->getParamNode()["obstacles"]["bbx18a"].as<int>();
          uint door2b = pParams->getParamNode()["obstacles"]["bbx18b"].as<int>();

          uint door3a = pParams->getParamNode()["obstacles"]["bbx19a"].as<int>();
          uint door3b = pParams->getParamNode()["obstacles"]["bbx19b"].as<int>();
          */

          // Setup 1: HR Closed and 1 NHR Closed
          //pNet->setObstacleStateValues(door2a, door2b, pMDP->getObstPenalty());
          //pNet->setObstacleStateValues(door3a, door3b, pMDP->getObstPenalty());

          // Setup 2: HR Closed and no NHR Closed
          //pNet->setObstacleStateValues(door2a, door2b, pMDP->getObstPenalty());

          // Setup 3: No HR Closed and 1 NHR Closed
          //pNet->setObstacleStateValues(door3a, door3b, pMDP->getObstPenalty());
/*

      }

      // id
    //    for (int i = pGrid2d->n_rows - 1; i >= 0; i--)
    //    {
    //      for (int j = 0; j < pGrid2d->n_cols; j++)
    //      {
    //        std::cout << pMDP->getpNet()->mdp_states[i * pGrid2d->n_cols + j]->id << " ";
    //      }
    //      std::cout << std::endl;
    //    }

      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      if(update == false)
      {
        &pMDP->iterations();
        &pMDP->optimalActionTransitionDistribution(&pNet->mdp_states);
      }
      else
      {
        &pMDP_buffer->iterations();
        &pMDP_buffer->optimalActionTransitionDistribution(&pNet->mdp_states);
      }
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>( t2 - t1 ).count();
      cout << "\nTime Taken: " << duration/1000000.0 << " seconds" << endl;

      if (hasObs) {

          pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));


      }
      //pGrid2d->cell_centers.clear();
      //pNet->mdp_states.clear();
      //delete &pDisturb;
      if(update == true)
      {
        //int size_buff = pGrid2d_buffer->cell_centers.size();
        std::cout<<"Im here in update = true"<<std::endl;
        //std::cout<<"buff_center"<<pGrid2d_buffer->cell_centers[0]<<std::endl;
        delete pGrid2d;
        delete pNet
        delete pMDP
        //pGrid2d->cell_centers.assign(pGrid2d_buffer->cell_centers.begin(),pGrid2d_buffer->cell_centers.end());
        //pNet->mdp_states.clear();
        update = false;
        //pNet->mdp_states.assign(pNet_buffer->mdp_states.begin(),pNet_buffer->mdp_states.end());
        //pGrid2d_buffer->cell_centers.clear();
        //pNet_buffer->cell_centers.clear();
      }
      else
      {
        std::cout<<"Im here in update = false"<<std::endl;
        delete pGrid2d_bffer;
        delete pNet_buffer;
        delete pMDP_buffer;
        //pGrid2d_buffer->cell_centers.clear();
        //pNet_buffer->mdp_states.clear();
        update = true;
      }
      //pNet->mdp_states = pNet_buffer->mdp_states;
      //pDisturb = pDisturb_buffer;
      std::cout<<"Im here finished assign value"<<std::endl;
      update = true;
    }

  // print map indices
  if(update == false)
  {
    std::cout << "\nmap indices" << std::endl;
    for (int i = pGrid2d -> n_rows - 1; i >= 0; i--) {
        for (int j = 0; j < pGrid2d -> n_cols; j++) {
            std::cout << i * pGrid2d -> n_cols + j << " ";
        }
        std::cout << std::endl;
    }
  }
  else
  {
    std::cout << "\nmap indices" << std::endl;
    for (int i = pGrid2d_buffer -> n_rows - 1; i >= 0; i--) {
        for (int j = 0; j < pGrid2d_buffer -> n_cols; j++) {
            std::cout << i * pGrid2d_buffer -> n_cols + j << " ";
        }
        std::cout << std::endl;
    }
  }


  vector<viz_tool::RGB> colors = viz_tool::generateRGB(tf2_starts.size(), 'r');
  for (uint i = 0; i < tf2_starts.size(); i++) {
    AUVmodel::Ptr auv = AUVmodel::Ptr(new AUVmodel(pParams, pDisturb, pNet));
    if (pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0 && !pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
      //pNet is generated inside MDPIteration
      auv->getpControl()->MDPIterations(tf2_goals);
      auv->getpControl()->getpNet()->setPolicyColor(colors[i]);
    }
    auv->getpControl()->getpNet()->getState(tf2_starts[i].translation)->type = START;
    auv->settf2(tf2_starts[i]);
    pRobots.push_back(auv);
  }
}
*/
void MethodManager::mdpCore(void){
  num_rows = pParams->getParamNode()["environment"]["grids"]["num_rows"].as<unsigned int>();
  num_cols = pParams->getParamNode()["environment"]["grids"]["num_cols"].as<unsigned int>();

  int resolution_layer = pParams->getParamNode()["mdp_methods"]["resolution_layer"].as<int>();;
  int num_rows_buff = num_rows/resolution_layer;
  int num_cols_buff = num_cols/resolution_layer;
  double resolution = 2.0 * bounds_xyz.x() / num_cols;
  mdp_state_t* goal;

  point2d_t origin(-bounds_xyz.x(), -bounds_xyz.y());
  std::vector<MDP_Grid2D::Ptr> pGrid2d_vector;
  std::vector<MDP::Ptr> pMDP_vector;
  std::vector<MDP_Net::Ptr> pNet_vector;
  std::vector<Disturbance::Ptr> pDisturb_vector;
  pGrid2d = MDP_Grid2D::Ptr(new MDP_Grid2D(num_cols, num_rows, resolution, origin));
  pNet = MDP_Net::Ptr(new MDP_Net(pGrid2d));
  pDisturb = Disturbance::Ptr(new Disturbance(pParams, pGrid2d));
  //Here is the resolution function I created**
  /*
  for(int scale = 0; scale < num_cols_buff-1; scale++)
  {
    cout<<"Im in"<<scale<<"initialization"<<endl;
  //double resolution = 2.0 * bounds_xyz.x() / num_cols;
  double resolution_buff = 2.0 * bounds_xyz.x() / (resolution_layer*(scale+1));

  cout << "Grid map dimension: " << num_rows_buff << "x" << num_cols_buff << "; origin: " << origin << endl;
  //pGrid2d_buff = std::vector<MDP_Grid2D::Ptr>(new MDP_Grid2D(num_cols_buff, num_rows_buff, resolution_buff, origin));
  //pGrid2d_buff = MDP_Grid2D::Ptr(new MDP_Grid2D(num_cols_buff, num_rows_buff, resolution_buff, origin));
  //pGrid2d_buff_vector.push_back(pGrid2d_buff);
  pGrid2d_buff = MDP_Grid2D::Ptr(new MDP_Grid2D(resolution_layer*(scale+1), resolution_layer*(scale+1), resolution_buff, origin));
  //pGrid2d_buff_vector.push_back(pGrid2d_buff);
  //cout<<"first is"<<pGrid2d_buff_vector[0]->cell_centers.size()<<endl;
  //cout<<"second is"<<pGrid2d_buff_vector[1]->cell_centers.size()<<endl;
  //pGrid2d_buff.push_back(new MDP_Grid2D(num_cols_buff, num_rows_buff, resolution_buff, origin));
  pNet_buff = MDP_Net::Ptr(new MDP_Net(pGrid2d_buff));
  pDisturb_buff = Disturbance::Ptr(new Disturbance(pParams, pGrid2d_buff));




  //cout << "Grid map dimension: " << num_rows << "x" << num_cols << "; origin: " << origin << endl;
  //pGrid2d = MDP_Grid2D::Ptr(new MDP_Grid2D(num_cols, num_rows, resolution, origin));
  //pNet = MDP_Net::Ptr(new MDP_Net(pGrid2d));
  //pDisturb = Disturbance::Ptr(new Disturbance(pParams, pGrid2d));



  if (pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0 && pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {

    pMDP_buff = MDP::Ptr(new MDP(pParams, pNet_buff, pDisturb_buff));

    // set the state type
    for (uint i = 0; i < tf2_starts.size(); i++) {
      pNet_buff->getState(tf2_starts[i].translation)->type = START;
    }
    for (uint i = 0; i < tf2_goals.size(); i++) {
      double goapNet->getState(tf2_starts[i].translation)l_value = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
      pNet_buff->getState(tf2_goals[i].translation)->type = GOAL;
      if(i==1){
          highgoalindex = i;
          pMDP_buff->fillTypeValue(pNet_buff->getState(tf2_goals[i].translation), GOAL, goal_value + 50);
      }
      else{
          pMDP_buff->fillTypeValue(pNet_buff->getState(tf2_goals[i].translation), GOAL, goal_value);
      }
    }
    pMDP_vector.push_back(pMDP_buff);
    pGrid2d_vector.push_back(pGrid2d_buff);
    pNet_vector.push_back(pNet_buff);
    pDisturb_vector.push_back(pDisturb_buff);
    }
  }
  */

  pMDP = MDP::Ptr(new MDP(pParams, pNet, pDisturb));

  // set the state type
  for (uint i = 0; i < tf2_starts.size(); i++) {
    pNet->getState(tf2_starts[i].translation)->type = START;
  }
  for (uint i = 0; i < tf2_goals.size(); i++) {
    double goal_value = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
    pNet->getState(tf2_goals[i].translation)->type = GOAL;
    if(i==1){
        highgoalindex = i;
        //pMDP->fillTypeValue(pNet->getState(tf2_goals[i].translation), GOAL, goal_value + 50);
    }
    //else{
      //  pMDP->fillTypeValue(pNet->getState(tf2_goals[i].translation), GOAL, goal_value);
    //}
}
  goal = pNet->getState(tf2_goals[0].translation);
  //std::cout<<"Im here at GMSPI"<<std::endl;
  //Policy_iteration(pNet->mdp_states,pNet);
  //mdp_planner::GMSPI(goal, pNet);
  //mdp_planner::Trail_Based_RTDP(pNet,pParams);

    // Including obstacles
    bool hasObs = pParams->getParamNode()["obstacles"]["hasObs"].as<bool>();
    if (hasObs) {
        //cout << "I am here 2.5" << endl;
        uint index1 = pParams->getParamNode()["obstacles"]["bbx1a"].as<int>();
        uint index2 = pParams->getParamNode()["obstacles"]["bbx1b"].as<int>();
        uint index3 = pParams->getParamNode()["obstacles"]["bbx2a"].as<int>();
        uint index4 = pParams->getParamNode()["obstacles"]["bbx2b"].as<int>();
        uint index5 = pParams->getParamNode()["obstacles"]["bbx3a"].as<int>();
        uint index6 = pParams->getParamNode()["obstacles"]["bbx3b"].as<int>();
        uint index7 = pParams->getParamNode()["obstacles"]["bbx4a"].as<int>();
        uint index8 = pParams->getParamNode()["obstacles"]["bbx4b"].as<int>();
        uint index9 = pParams->getParamNode()["obstacles"]["bbx5a"].as<int>();
        uint index10 = pParams->getParamNode()["obstacles"]["bbx5b"].as<int>();
        uint index11 = pParams->getParamNode()["obstacles"]["bbx6a"].as<int>();
        uint index12 = pParams->getParamNode()["obstacles"]["bbx6b"].as<int>();
        uint index13 = pParams->getParamNode()["obstacles"]["bbx7a"].as<int>();
        uint index14 = pParams->getParamNode()["obstacles"]["bbx7b"].as<int>();
        uint index15 = pParams->getParamNode()["obstacles"]["bbx8a"].as<int>();
        uint index16 = pParams->getParamNode()["obstacles"]["bbx8b"].as<int>();
        uint index17 = pParams->getParamNode()["obstacles"]["bbx9a"].as<int>();
        uint index18 = pParams->getParamNode()["obstacles"]["bbx9b"].as<int>();
        uint index19 = pParams->getParamNode()["obstacles"]["bbx10a"].as<int>();
        uint index20 = pParams->getParamNode()["obstacles"]["bbx10b"].as<int>();
        uint index21 = pParams->getParamNode()["obstacles"]["bbx11a"].as<int>();
        uint index22 = pParams->getParamNode()["obstacles"]["bbx11b"].as<int>();
        uint index23 = pParams->getParamNode()["obstacles"]["bbx12a"].as<int>();
        uint index24 = pParams->getParamNode()["obstacles"]["bbx12b"].as<int>();
        uint index25 = pParams->getParamNode()["obstacles"]["bbx13a"].as<int>();
        uint index26 = pParams->getParamNode()["obstacles"]["bbx13b"].as<int>();
        uint index27 = pParams->getParamNode()["obstacles"]["bbx14a"].as<int>();
        uint index28 = pParams->getParamNode()["obstacles"]["bbx14b"].as<int>();
        uint index29 = pParams->getParamNode()["obstacles"]["bbx15a"].as<int>();
        uint index30 = pParams->getParamNode()["obstacles"]["bbx15b"].as<int>();
        uint index31 = pParams->getParamNode()["obstacles"]["bbx16a"].as<int>();
        uint index32 = pParams->getParamNode()["obstacles"]["bbx16b"].as<int>();

        uint index33 = pParams->getParamNode()["obstacles"]["bbx20a"].as<int>();
        uint index34 = pParams->getParamNode()["obstacles"]["bbx20b"].as<int>();
        uint index35 = pParams->getParamNode()["obstacles"]["bbx21a"].as<int>();
        uint index36 = pParams->getParamNode()["obstacles"]["bbx21b"].as<int>();
        uint index37 = pParams->getParamNode()["obstacles"]["bbx22a"].as<int>();
        uint index38 = pParams->getParamNode()["obstacles"]["bbx22b"].as<int>();
        uint index39 = pParams->getParamNode()["obstacles"]["bbx23a"].as<int>();
        uint index40 = pParams->getParamNode()["obstacles"]["bbx23b"].as<int>();
        uint index41 = pParams->getParamNode()["obstacles"]["bbx24a"].as<int>();
        uint index42 = pParams->getParamNode()["obstacles"]["bbx24b"].as<int>();

        /*pNet->setObstacleStateValues(index1, index2, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index3, index4, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index5, index6, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index7, index8, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index9, index10, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index11, index12, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index13, index14, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index15, index16, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index17, index18, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index19, index20, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index21, index22, pMDP->getObstPenalty());*/
        //pNet->setObstacleStateValues(index23, index24, pMDP->getObstPenalty());
        //pNet->setObstacleStateValues(index25, index26, pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index27, index28, 0);//pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index29, index30, 0);//pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index31, index32, 0);//pMDP->getObstPenalty());

        pNet->setObstacleStateValues(index33, index34, 0);//pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index35, index36, 0);//pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index37, index38, 0);//pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index39, index40, 0);//pMDP->getObstPenalty());
        pNet->setObstacleStateValues(index41, index42, 0);//pMDP->getObstPenalty());


        /*
        uint door1a = pParams->getParamNode()["obstacles"]["bbx17a"].as<int>();
        uint door1b = pParams->getParamNode()["obstacles"]["bbx17b"].as<int>();

        uint door2a = pParams->getParamNode()["obstacles"]["bbx18a"].as<int>();
        uint door2b = pParams->getParamNode()["obstacles"]["bbx18b"].as<int>();

        uint door3a = pParams->getParamNode()["obstacles"]["bbx19a"].as<int>();
        uint door3b = pParams->getParamNode()["obstacles"]["bbx19b"].as<int>();
        */

        // Setup 1: HR Closed and 1 NHR Closed
        //pNet->setObstacleStateValues(door2a, door2b, pMDP->getObstPenalty());
        //pNet->setObstacleStateValues(door3a, door3b, pMDP->getObstPenalty());

        // Setup 2: HR Closed and no NHR Closed
        //pNet->setObstacleStateValues(door2a, door2b, pMDP->getObstPenalty());

        // Setup 3: No HR Closed and 1 NHR Closed
        //pNet->setObstacleStateValues(door3a, door3b, pMDP->getObstPenalty());


    }


    // id
  //    for (int i = pGrid2d->n_rows - 1; i >= 0; i--)
  //    {
  //      for (int j = 0; j < pGrid2d->n_cols; j++)
  //      {
  //        std::cout << pMDP->getpNet()->mdp_states[i * pGrid2d->n_cols + j]->id << " ";
  //      }
  //      std::cout << std::endl;
  //    }

/*
    high_resolution_clock::time_point t1_buff = high_resolution_clock::now();
    pMDP_buff->iterations();
    pMDP_buff->optimalActionTransitionDistribution(pNet_buff->mdp_states);
    high_resolution_clock::time_point t2_buff = high_resolution_clock::now();
    auto duration_buff = duration_cast<microseconds>( t2_buff - t1_buff ).count();
    cout << "\nTime Taken buff: " << duration_buff/1000000.0 << " seconds" << endl;
*/
    bool pass_down_policy = pParams->getParamNode()["mdp_methods"]["pass_down_policy"].as<bool>();


    if(pass_down_policy == true)
    {
      for(int scale2=0; scale2 < num_cols_buff-1; scale2++)
      {
        cout<<"Im in layer"<<scale2<<endl;
        high_resolution_clock::time_point t1_buff = high_resolution_clock::now();
        pMDP_vector[scale2]->iterations();
        pMDP_vector[scale2]->optimalActionTransitionDistribution(pNet_vector[scale2]->mdp_states);
        high_resolution_clock::time_point t2_buff = high_resolution_clock::now();
        auto duration_buff = duration_cast<microseconds>( t2_buff - t1_buff ).count();
        cout << "\nTime Taken buff: " << duration_buff/1000000.0 << " seconds" << endl;
        if(scale2 == num_cols_buff-2)
        {
          break;
        }
      for(uint i=0; i<(int)pGrid2d_vector[scale2+1]->cell_centers.size();i++)
      {
        //std::cout<<"IM here in first for"<<std::endl;
        int index = -1;
        float distance_optimal = 20000;
        for(uint j=0; j<(int)pGrid2d_vector[scale2]->cell_centers.size();j++)
        {
          //std::cout<<"Im here in second for"<<std::endl;
          float x_dist = pGrid2d_vector[scale2+1]->cell_centers[i][0]-pGrid2d_vector[scale2]->cell_centers[j][0];
          //std::cout<<"x_dist"<<x_dist<<std::endl;
          float y_dist = pGrid2d_vector[scale2+1]->cell_centers[i][1]-pGrid2d_vector[scale2]->cell_centers[j][1];
          //std::cout<<"y_dist"<<y_dist<<std::endl;
          float distance = std::sqrt(x_dist*x_dist+y_dist*y_dist);
          //std::cout<<"This is distance"<<distance<<std::endl;
          if(distance < distance_optimal)
          {
            distance_optimal = distance;
            index = j;
          }
        }
        //std::cout<<"index is"<<index<<endl;
        if(index == -1)
        {
          continue;
        }
        pNet_vector[scale2+1]->mdp_states[i]->optimal_action = pNet_vector[scale2]->mdp_states[index]->optimal_action;
      }
    }
    }

/*
    if(pass_down_policy == true)
    {
      for(int scale2=0; scale < num_cols_buff-1; scale++)
      {

      for(uint i=0; i<(int)pGrid2d->cell_centers.size();i++)
      {
        //std::cout<<"IM here in first for"<<std::endl;
        int index = -1;
        float distance_optimal = 20000;
        for(uint j=0; j<(int)pGrid2d_buff->cell_centers.size();j++)
        {
          //std::cout<<"Im here in second for"<<std::endl;
          float x_dist = pGrid2d->cell_centers[i][0]-pGrid2d_buff->cell_centers[j][0];
          //std::cout<<"x_dist"<<x_dist<<std::endl;
          float y_dist = pGrid2d->cell_centers[i][1]-pGrid2d_buff->cell_centers[j][1];
          //std::cout<<"y_dist"<<y_dist<<std::endl;
          float distance = std::sqrt(x_dist*x_dist+y_dist*y_dist);
          //std::cout<<"This is distance"<<distance<<std::endl;
          if(distance < distance_optimal)
          {
            distance_optimal = distance;
            index = j;
          }
        }
        //std::cout<<"index is"<<index<<endl;
        if(index == -1)
        {
          continue;
        }
        pNet->mdp_states[i]->optimal_action = pNet_buff->mdp_states[index]->optimal_action;
      }
    }
    }
    */

      //std::cout<<"This is index"<<index<<std::endl;
      //std::cout<<"Im here in assign optimal value"<<std::endl;
      //pNet_buffer->mdp_states[i]->optimal_value = pNet->mdp_states[index]->optimal_value;

    cout<<"Im in computing pMDP"<<endl;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    if(pass_down_policy == true)
    {
      for(uint i=0; i<(int)pGrid2d->cell_centers.size();i++)
      {
        //std::cout<<"IM here in first for"<<std::endl;
        int index = -1;
        float distance_optimal = 20000;
        for(uint j=0; j<(int)pGrid2d_vector[num_cols_buff-2]->cell_centers.size();j++)
        {
          //std::cout<<"Im here in second for"<<std::endl;
          float x_dist = pGrid2d->cell_centers[i][0]-pGrid2d_vector[num_cols_buff-2]->cell_centers[j][0];
          //std::cout<<"x_dist"<<x_dist<<std::endl;
          float y_dist = pGrid2d->cell_centers[i][1]-pGrid2d_vector[num_cols_buff-2]->cell_centers[j][1];
          //std::cout<<"y_dist"<<y_dist<<std::endl;
          float distance = std::sqrt(x_dist*x_dist+y_dist*y_dist);
          //std::cout<<"This is distance"<<distance<<std::endl;
          if(distance < distance_optimal)
          {
            distance_optimal = distance;
            index = j;
          }
        }
        //std::cout<<"index is"<<index<<endl;
        if(index == -1)
        {
          continue;
        }
        pNet->mdp_states[i]->optimal_action = pNet_vector[num_cols_buff-2]->mdp_states[index]->optimal_action;
      }
    }
    //mdp_planner::Trail_Based_RTDP(pNet,pParams);
    mdp_planner::MFPT_RTDP(pNet,pParams,pDisturb);
    //mdp_planner::Policy_iteration(pNet->mdp_states,pNet);
    //mdp_planner::GMSPI(goal, pNet);
    //pMDP->iterations();
    //states_fpt_final = pMDP->states_fpt_final;
    pMDP->optimalActionTransitionDistribution(pNet->mdp_states);
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    cout << "\nTime Taken: " << duration/1000000.0 << " seconds" << endl;

    if (hasObs) {

        pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

  //        double originX = 0.0, originY = 0.0, radius = 2, angle = 0.628;
  //        for(int i = 1; i <= 10; i++){
  //            double x = originX + cos(angle*i)*radius;
  //            double y = originY + sin(angle*i)*radius;

  //            cout << "X: " << x << " , Y: " << y << endl;
  //        }

        //uint index41 = pParams->getParamNode()["obstacles"]["bbx24a"].as<int>();
        //uint index42 = pParams->getParamNode()["obstacles"]["bbx24b"].as<int>();
        //uint index43 = pParams->getParamNode()["obstacles"]["bbx25a"].as<int>();
        //uint index44 = pParams->getParamNode()["obstacles"]["bbx25b"].as<int>();


        //pNet->reachablestates.clear();
        //pNet->setObstacleStateValues(index41, index42, pMDP->getObstPenalty());
        //pNet->setObstacleStateValues(index43, index44, pMDP->getObstPenalty());

  //        high_resolution_clock::time_point t1 = high_resolution_clock::now();
  //        pMDP->optimalActionTransitionDistribution(pNet->mdp_states);
  //        pSSP->initTransMatrix();
  //        double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts[0].translation), pNet->getState(tf2_goals[0].translation)->id);


  //        pMDP->iterations();
  //        high_resolution_clock::time_point t2 = high_resolution_clock::now();
  //        auto duration = duration_cast<microseconds>( t2 - t1 ).count();
  //        cout << "\nTime Taken: " << duration/1000000.0 << " seconds" << endl;

        //pMDP->optimalActionTransitionDistribution(pNet->mdp_states);

        /*std::cout << "Unreachable states to High Goal: " << pNet->reachablestates.size() << std::endl;

        if(true)
        {
            std::cout << "Need to update policy locally" << std::endl;

            uint num_iterations = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
            for (uint i = 0; i < num_iterations; i++)
            {
                cout << i << " " << std::flush;
                pMDP->valueIterationLocalStates(pNet->mdp_states,pNet->reachablestates,"forward");
            }
            //pMDP->optimalActionTransitionDistribution(pNet->mdp_states);

        }
        else{
            std::cout << "No need to update policy" << std::endl;
        }*/



        //closeDoor(1);
        //closeDoor(2);

        //openDoor(1);
        //openDoor(2);

        //removeGoal(0);
        //removeGoal(1);
        //removeGoal(2);


    }
  ofstream my_file2;

  // print map indices
  my_file2.open("state_value.txt");
  std::cout << "\nmap indices" << std::endl;
  for (int i = pGrid2d -> n_rows - 1; i >= 0; i--) {
      for (int j = 0; j < pGrid2d -> n_cols; j++) {
          //std::cout << i * pGrid2d -> n_cols + j << " ";
          std::cout<< (int)pNet->mdp_states[i*pGrid2d->n_cols+j]->optimal_value<<" ";
          my_file2<<pNet->mdp_states[i*pGrid2d->n_cols+j]->optimal_value<<",";
      }
      std::cout << std::endl;
  }
  my_file2.close();

  vector<viz_tool::RGB> colors = viz_tool::generateRGB(tf2_starts.size(), 'r');

  for (uint i = 0; i < tf2_starts.size(); i++) {
    AUVmodel::Ptr auv = AUVmodel::Ptr(new AUVmodel(pParams, pDisturb, pNet));
    if (pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0 && !pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
      //pNet is generated inside MDPIteration
      auv->getpControl()->MDPIterations(tf2_goals);
      auv->getpControl()->getpNet()->setPolicyColor(colors[i]);
    }
    auv->getpControl()->getpNet()->getState(tf2_starts[i].translation)->type = START;
    auv->settf2(tf2_starts[i]);
    pRobots.push_back(auv);
  }

}
void
MethodManager::mdpImportedVecField(void) {
  pData = utils::DataLoader::Ptr(new utils::DataLoader(pParams));
  pData->importVecFieldData();

  vector<double> longitudes = pData->getLongitudes(),
          latitudes = pData->getLatitudes();
  double x_min = *std::min_element(longitudes.begin(), longitudes.end());
  double x_max = *std::max_element(longitudes.begin(), longitudes.end());
  double y_min = *std::min_element(latitudes.begin(), latitudes.end());
  double y_max = *std::max_element(latitudes.begin(), latitudes.end());

  double x_length = x_max - x_min;
  double y_length = y_max - y_min;

  point2d_t origin(-bounds_xyz.x(), -bounds_xyz.y());
  double ratio_x = 2 * bounds_xyz.x() / x_length;
  double ratio_y = 2 * bounds_xyz.y() / y_length;
  num_cols = longitudes.size();
  num_rows = latitudes.size();
  double resolution = 2 * bounds_xyz.x() / std::max(num_cols, num_rows);

  cout << "Grid map dimension: " << num_rows << "x" << num_cols << "; origin: " << origin << endl;
  pGrid2d = MDP_Grid2D::Ptr(new MDP_Grid2D(num_cols, num_rows, resolution, origin));

  pDisturb = Disturbance::Ptr(new Disturbance(pParams, pData, pGrid2d));

  pNet = MDP_Net::Ptr(new MDP_Net(pGrid2d));
  // pNet and obst setting is commonly used by shared and non-shared policy
  pNet->setObstacleStateValues(pData->getVecField(), pParams->getParamNode()["mdp_methods"]["obst_penalty"].as<double>());

  if (pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0 && pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
    pMDP = MDP::Ptr(new MDP(pParams, pNet, pDisturb));

    // set the state type
    for (uint i = 0; i < tf2_starts.size(); i++) {
      pNet->getState(tf2_starts[i].translation)->type = START;
    }
    Transform2 goal_tf2;
    goal_tf2.translation = origin + Vec2(0.2 * x_length*ratio_x, 0.6 * y_length * ratio_y); //goal inside the field
    pNet->getState(goal_tf2.translation)->type = GOAL;
    double goal_value = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
    pMDP->fillTypeValue(pMDP->getpNet()->getState(goal_tf2.translation), GOAL, goal_value);

    pMDP->iterations();
  }

  vector<viz_tool::RGB> colors = viz_tool::generateRGB(tf2_starts.size(), 'r');
  for (uint i = 0; i < tf2_starts.size(); i++) {
    AUVmodel::Ptr auv = AUVmodel::Ptr(new AUVmodel(pParams, pDisturb, pNet));
    if (pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0 && !pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
      //re-set goals if necessary, like this:
      Transform2 goal_tf2;
      goal_tf2.translation = origin + Vec2(0.2 * x_length*ratio_x, 0.6 * y_length * ratio_y); //goal inside the field
      tf2_goals.clear();
      tf2_goals.push_back(goal_tf2);
      //pNet is generated inside MDPIteration
      auv->getpControl()->MDPIterations(tf2_goals);
      auv->getpControl()->getpNet()->setPolicyColor(colors[i]);
    }
    auv->getpControl()->getpNet()->getState(tf2_starts[i].translation)->type = START;
    auv->settf2(tf2_starts[i]);
    pRobots.push_back(auv);
  }

}

void
MethodManager::infoPlan(void) {
  pData = utils::DataLoader::Ptr(new utils::DataLoader(pParams));
  pData->importVecFieldData();

  vector<double> longitudes = pData->getLongitudes();
  vector<double> latitudes = pData->getLatitudes();
  double x_min = *std::min_element(longitudes.begin(), longitudes.end());
  double x_max = *std::max_element(longitudes.begin(), longitudes.end());
  double y_min = *std::min_element(latitudes.begin(), latitudes.end());
  double y_max = *std::max_element(latitudes.begin(), latitudes.end());

  double x_length = x_max - x_min;
  double y_length = y_max - y_min;
  point2d_t origin(-bounds_xyz.x(), -bounds_xyz.y());
  double ratio_x = 2 * bounds_xyz.x() / x_length;
  double ratio_y = 2 * bounds_xyz.y() / y_length;
  num_cols = longitudes.size();
  num_rows = latitudes.size();
  double resolution = 2 * bounds_xyz.x() / std::max(num_cols, num_rows);

  cout << "Grid map dimension: " << num_rows << "x" << num_cols << "; origin: " << origin << endl;

  pGrid2d = MDP_Grid2D::Ptr(new MDP_Grid2D(num_cols, num_rows, resolution, origin));
  pDisturb = Disturbance::Ptr(new Disturbance(pParams, pData, pGrid2d));
  pNet = MDP_Net::Ptr(new MDP_Net(pGrid2d));

  // pNet and obst setting is commonly used by shared and non-shared policy
  pNet->setObstacleStateValues(pData->getVecField(), pParams->getParamNode()["mdp_methods"]["obst_penalty"].as<double>());

  dir_name = "results";
  if (boost::filesystem::create_directory(dir_name)) {
    std::cout << "Success" << "\n";
  }

  // Choosing info solver
  Solver_Interface *solver = nullptr;
  const std::string solver_type = pParams->getParamNode()["info_plan"]["solver"].as<std::string>();
  if (solver_type == "dp") {
    solver = new DP_Solver();
  } else if (solver_type == "greedy") {
    solver = new Greedy_Solver();
  } else {
    std::cerr << "Wrong Info solver" << std::endl;
  }

  // Run info method
  info_paths paths;
  if (solver) {
    const size_t stages = pParams->getParamNode()["info_plan"]["stages"].as<size_t>();
    const size_t layers = pParams->getParamNode()["info_plan"]["layers"].as<size_t>();
    const size_t grid_x_size = pParams->getParamNode()["info_plan"]["grid_x_size"].as<size_t>();
    const size_t grid_y_size = pParams->getParamNode()["info_plan"]["grid_y_size"].as<size_t>();

    // Initial hyperparameters
    std::vector<double> cov_params;
    cov_params.push_back(pParams->getParamNode()["info_plan"]["gp_params"]["cov_params1"].as<double>());
    cov_params.push_back(pParams->getParamNode()["info_plan"]["gp_params"]["cov_params2"].as<double>());
    cov_params.push_back(pParams->getParamNode()["info_plan"]["gp_params"]["cov_params3"].as<double>());
    cov_params.push_back(pParams->getParamNode()["info_plan"]["gp_params"]["cov_params4"].as<double>());

    utils::nc_data_t nc_data = pData->getRawNCdata();

    // Add observations
    std::set<gp_point> observations;
//    observations.insert(gp_point(0, nc_data.n_cols - 1));
//    observations.insert(gp_point(nc_data.n_rows - 1, 0));

    std::vector<gp_point> starting_points;
    for (size_t i = 0; i < tf2_starts.size(); i++) {
      gp_point starting_point;
      point2d_t start = tf2_starts[i].translation - origin;
      starting_point.first = std::round(static_cast<double> (start.y() / static_cast<double> (2 * bounds_xyz.y()) * static_cast<double> (nc_data.n_rows)));
      starting_point.second = std::round(static_cast<double> (start.x() / static_cast<double> (2 * bounds_xyz.x()) * static_cast<double> (nc_data.n_cols)));
      starting_points.push_back(starting_point);
    }

    srand(time(NULL));

    for (size_t i = 0; i < 1; i++) {
      // Add random observations
//      std::vector<Vec2i> observations;
//      int x = rand() % nc_data.n_rows;
//      int y = rand() % nc_data.n_cols;
//      while (std::isnan(nc_data.frames[0].cells[x][y].salinity)) {
//        x = rand() % nc_data.n_rows;
//        y = rand() % nc_data.n_cols;
//      }
//      observations.push_back(Vec2i(x, y));
//      x = rand() % nc_data.n_rows;
//      y = rand() % nc_data.n_cols;
//      while (std::isnan(nc_data.frames[0].cells[x][y].salinity)) {
//        x = rand() % nc_data.n_rows;
//        y = rand() % nc_data.n_cols;
//      }
//      observations.push_back(Vec2i(x, y));

      pInfo = info_planner::Info::Ptr(new info_planner::Info(pParams, cov_params, layers, grid_x_size, grid_y_size, nc_data, starting_points, observations, solver));
      pInfo->num_robots = tf2_starts.size();
      for (size_t j = stages; j <= stages; j += 5) {
        paths = pInfo->run(nc_data, 0, j, i);
        pInfo->pred(paths);
      }
    }

    // Transform paths into goals
    for (size_t i = 0; i < tf2_starts.size(); i++) {
      std::queue<Transform2> goals;
      for (size_t j = 0; j < paths[i].size(); j++) {
        double scale_x = paths[i][j].second / static_cast<double> (nc_data.n_cols);
        double scale_y = paths[i][j].first / static_cast<double> (nc_data.n_rows);
        Vec2 pos = origin + Vec2(scale_x * x_length * ratio_x, scale_y * y_length * ratio_y);
        goals.push(Transform2(pos));
      }
      pInfo->setGoals(goals, i);
    }

    // shared policy
    if (pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0 && pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
      pMDP = MDP::Ptr(new MDP(pParams, pNet, pDisturb));

      // set the state type
      for (size_t i = 0; i < tf2_starts.size(); i++) {
        pNet->getState(tf2_starts[i].translation)->type = START;

        Transform2 goal_tf2 = pInfo->getGoals(i).front();
        pNet->getState(goal_tf2.translation)->type = GOAL;
        double goal_value = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
        pMDP->fillTypeValue(pMDP->getpNet()->getState(goal_tf2.translation), GOAL, goal_value);
      }

      pMDP->iterations();
    }

    // independent policy
    // TODO:different color for each auv
    vector<viz_tool::RGB> colors;
    colors.push_back(viz_tool::generateRGB(1, 'r')[0]);
    colors.push_back(viz_tool::generateRGB(1, 'b')[0]);
    for (size_t i = 0; i < tf2_starts.size(); i++) {
      AUVmodel::Ptr auv = AUVmodel::Ptr(new AUVmodel(pParams, pDisturb, pNet));
      auv->getpControl()->getpNet()->getState(tf2_starts[i].translation)->type = START;
      if (pParams->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0 && !pParams->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
        tf2_goals.clear();
        tf2_goals.push_back(pInfo->getGoals(i).front());
        // pNet is generated inside MDPIteration
        auv->getpControl()->MDPIterations(tf2_goals);
        auv->getpControl()->getpNet()->setPolicyColor(colors[i]);
      }
      auv->settf2(tf2_starts[i]);
      pRobots.push_back(auv);
    }

    const utils::nc_data_t mdp_nc_data = pData->getNCdata();
    pInfo->setMdpMapSize(mdp_nc_data.n_rows, mdp_nc_data.n_cols);

    delete solver;
  }
}

void
MethodManager::mdpExpectedSSP(void) {
    // TODO
    //cout << "I am here" << endl;
    num_rows = pParams->getParamNode()["environment"]["grids"]["num_rows"].as<unsigned int>();
    num_cols = pParams->getParamNode()["environment"]["grids"]["num_cols"].as<unsigned int>();

    point2d_t origin(-bounds_xyz.x(), -bounds_xyz.y());
    double resolution = 2.0 * bounds_xyz.x() / num_cols;

    cout<<"Grid map dimension: "<<num_rows<<"x"<<num_cols<<"; origin: "<<origin<<endl;
    pGrid2d = MDP_Grid2D::Ptr(new MDP_Grid2D(num_cols, num_rows, resolution, origin));
    pDisturb = Disturbance::Ptr(new Disturbance(pParams, pGrid2d));
    //cout << "I am here 2" << endl;
    pNet = MDP_Net::Ptr(new MDP_Net(pGrid2d));

    pDisturb = Disturbance::Ptr(new Disturbance(pParams, pGrid2d));

    pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

    bool hasObs = pParams->getParamNode()["obstacles"]["hasObs"].as<bool>();
    if (hasObs) {
        //cout << "I am here 2.5" << endl;
        uint index1 = pParams->getParamNode()["obstacles"]["bbx1a"].as<int>();
        uint index2 = pParams->getParamNode()["obstacles"]["bbx1b"].as<int>();
        uint index3 = pParams->getParamNode()["obstacles"]["bbx2a"].as<int>();
        uint index4 = pParams->getParamNode()["obstacles"]["bbx2b"].as<int>();
        //cout << "I am here 2.6" << endl;
        pNet->setObstacleStateValues(index1, index2, pSSP->getObstPenalty());
        //cout << "I am here 2.7" << endl;
        pNet->setObstacleStateValues(index3, index4, pSSP->getObstPenalty());
        //cout << "I am here 2.8" << endl;
    }
    //cout << "I am here 3" << endl;
    std::queue<Transform2> goals;
    goals.push(Transform2(Vec2(-10, 1)));
    goals.push(Transform2(Vec2(10, 10)));

    pSSP->setGoals(goals);

    // comupting optimal policy
    pSSP->iterations();
    //cout << "I am here 3.1" << endl;
    pSSP->optimalActionTransitionDistribution(pNet->mdp_states);
    //cout << "I am here 3.2" << endl;
    pSSP->initTransMatrix();

    // print map indices
    std::cout << "map indices" << std::endl;
    for (int i = pGrid2d -> n_rows - 1; i >= 0; i--) {
        for (int j = 0; j < pGrid2d -> n_cols; j++) {
            std::cout << i * pGrid2d -> n_cols + j << " ";
        }
        std::cout << std::endl;
    }

    //cout << "I am here 4" << endl;
    //calculate mean first passage time
    pSSP->meanFirstPassageTime(pNet->getState(tf2_starts[0].translation), pNet->getState(tf2_goals[0].translation)->id);

    //cout << "I am here 5" << endl;
    // //Monte Carlo simulation
    //  std::cout << "mcFirstPassageTime " << std::endl;
    //  for (int i = pGrid2d -> n_rows - 1; i >= 0; i--) {
    //    for (int j = 0; j < pGrid2d -> n_cols; j++) {
    //      double fpt = pSSP->mcFirstPassageTime(pSSP->getGridPtr()->getState(tf2_start.translation), i * pGrid2d -> n_cols + j);
    //      //      std::cout << fpt << " ";
    //    }
    //    //    std::cout << std::endl;
    //  }

    //set the state type
    for (uint i = 0; i < tf2_starts.size(); i++) {
        pNet->getState(tf2_starts[i].translation)->type = START;
    }

    for (uint i = 0; i < tf2_goals.size(); i++) {
        double goal_value = 100;
        pNet->getState(tf2_goals[i].translation)->type = GOAL;
        pSSP->fillTypeValue(pNet->getState(tf2_goals[i].translation), GOAL, goal_value);
    }

    //    double fpt = pSSP->mcFirstPassageTime(pSSP->getGridPtr()->getState(tf2_start.translation)->id, pSSP->getGridPtr()->getState(tf2_goal.translation)->id);
    //    std::cout << "Monte Carlo from " << pSSP->getGridPtr()->getState(tf2_start.translation)->id << " to " << pSSP->getGridPtr()->getState(tf2_goal.translation)->id << ": "<< fpt << std::endl;
    //    fpt = pSSP->mcFirstPassageTime(190, pSSP->getGridPtr()->getState(tf2_goal.translation)->id);
    //    std::cout << "Monte Carlo from " << 190 << " to " << pSSP->getGridPtr()->getState(tf2_goal.translation)->id << ": "<< fpt << std::endl;

    for (uint i = 0; i < tf2_starts.size(); i++) {
        AUVmodel::Ptr auv = AUVmodel::Ptr(new AUVmodel(pParams, pDisturb, pNet));

        vector<mdp_state_t*> ws =
                pSSP->SSP::getExpectedWayStates(pNet->getState(tf2_starts[i].translation));

        vector<Transform2> tmp_waypts;
        if (pParams->getParamNode()["expected_ssp"]["spline_fit"].as<bool>()) {
            vector<Transform2> mod_waypts = pSSP->pathSharpTurnsRemoval(pSSP->getWaypoints(ws), 91, 0);
            tmp_waypts = pSSP->convertSplinePaths2Waypoints(pSSP->splinePaths(mod_waypts));
        } else {
            tmp_waypts = pSSP->getWaypoints(ws);
        }

        auv->getpControl()->waypoints.clear();
        for (uint j = 0; j < tmp_waypts.size(); j++) {
            auv->getpControl()->waypoints.push_back(tmp_waypts[j]);
        }
        cout << "#waypoints: " << tmp_waypts.size() << endl;

        if (pParams->getParamNode()["expected_ssp"]["spline_fit"].as<bool>()) {
            vector<Transform2> mod_waypts = pSSP->pathSharpTurnsRemoval(pSSP->getWaypoints(ws), 91, 0);
            tmp_waypts = pSSP->convertSplinePaths2Waypoints(pSSP->splinePaths(mod_waypts));
        } else {
            tmp_waypts = pSSP->getWaypoints(ws);
        }

        auv->getpControl()->waypoints.clear();
        for (uint j = 0; j < tmp_waypts.size(); j++) {
            auv->getpControl()->waypoints.push_back(tmp_waypts[j]);
        }
        cout << "#waypoints: " << tmp_waypts.size() << endl;

        auv->settf2(auv->getpControl()->waypoints.front());

        pRobots.push_back(auv);

    }

}
