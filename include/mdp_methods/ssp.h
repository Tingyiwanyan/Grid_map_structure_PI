/*
Stochastic Shortest Path (SSP)
 */

#ifndef STOCH_SHORTEST_PATH_H
#define STOCH_SHORTEST_PATH_H

#include "mdp_core.h"
#include <queue>
#include <map>
#include <Eigen/Sparse>
#include <algorithm>

using namespace std;
using namespace geometry_utils;


namespace mdp_planner {

  class SSP : public MDP {
  public:
    typedef Eigen::SparseMatrix<double> SpMat;
    //	typedef Eigen::SparseVector<double> SpVec;

    typedef boost::shared_ptr<SSP> Ptr;
    typedef boost::shared_ptr<const SSP> ConstPtr;

    vector<pair<double,int>> sweepedStates;
    int loopind;

    SSP(const utils::Parameters::Ptr&, const MDP_Net::Ptr&, const Disturbance::Ptr&);
    virtual ~SSP();

    //accessors
    //virtual void loadParams(void){};

    // track the state and concatinate them to get a sequence of states
    // such "future path" is in the expected sense, can be "misleading" though
    static vector<mdp_state_t*>
    getExpectedWayStates(mdp_state_t* _start, MDP_Net::Ptr&);
    vector<mdp_state_t*>
    getExpectedWayStates(mdp_state_t* _start);

    static vector<mdp_state_t*>
    getExpectedWayStates(mdp_state_t* _start, mdp_state_t* _goal, MDP_Net::Ptr&);
    vector<mdp_state_t*>
    getExpectedWayStates(mdp_state_t* _start, mdp_state_t* _goal);

    //convert waystates to waypoints (poses)
    static vector<Transform2>
    getWaypoints(const vector<mdp_state_t*>& _waystates, MDP_Net::Ptr&);
    vector<Transform2>
    getWaypoints(const vector<mdp_state_t*>& _waystates);

    //smooth a path by removing sharp turn waypoints, rule:
    // (1) each such node of angle within _angle_threshold,
    // (2) and the _n/2 nodes before it, and the _n-_n/2 nodes after it
    vector<Transform2>
    pathSharpTurnsRemoval(const vector<Transform2>& _in_paths,
            int _angle_threshold,
            int _num_nodes);


    //spline a path of way_states, each output arma::vec denotes a path
    vector<arma::vec>
    splinePaths(const vector<Transform2>& _ws);

    //from arma paths output to waypoints
    vector<Transform2>
    convertSplinePaths2Waypoints(const vector<arma::vec>& _paths);

    /* other util functions */
    //convert arma vec to STL vector or deque

    template <typename T>
    void convertArmaVecTo(const arma::vec& _v, T& _t) {
      _t.clear();
      for (uint i = 0; i < _v.n_elem; i++)
        _t.push_back(_v[i]);
    }

    //get vec of isolated x/y/z from wayponts, contents determined by _label
    vector<double>
    getWaypoints1D(const vector<Transform2>& _wp, char _label);

    //calculate knots/intervels times based on waypoints and const speed
    vector<double>
    getKnotTimes(const vector<Transform2>& _waypoints, double _speed);

    void fillTransMatrix(SpMat& matrix, const int &state_id, const int &cols, const vector<double>& probs,int oo);

    // fill in the global transition probability (size: grids * grids)
    void initTransMatrix();

    void printTransMatrix(const int &state_i, const int &state_j);

    double meanFirstPassageTime(const mdp_state_t* start_state, const int &end_state);
    void MFPT(MDP_Net::Ptr& pNet, int id);
    void MFPT_obstacle(MDP_Net::Ptr& pNet);


    double mcFirstPassageTime(const int &start_state, const int &end_state);


    vector<double> getBel(Eigen::SparseMatrix<double> belief);
    vector<vector<double> > getBels();

    void setGoals(const std::queue<Transform2>& goals) {
      q_goals = goals;
    }

    std::queue<Transform2>& getGoals() {
      return q_goals;
    }

  //private:
    // n-step transition probability matrix p_ij(k)
    vector<Eigen::SparseMatrix<double> > transMatrixs;
    // transition probability matrix p_ij(k) at each step
    Eigen::SparseMatrix<double> transMatrix;
    // state distribution
    Eigen::SparseMatrix<double> bel;
    // state distribution all steps, express in vector
    vector<vector<double> > bels;
    vector<double> f_ij;
    double *reachability;

    std::queue<Transform2> q_goals;
  };

}


#endif
