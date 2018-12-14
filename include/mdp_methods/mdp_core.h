#ifndef MDP_CORE_H
#define MDP_CORE_H

#include <assert.h>
#include <iterator>
#include <algorithm>

#include "geometry_utils/Transform2.h"
#include "spline/UnconstrainedSplineFit.h"
#include "utils/parameters.h"
#include "utils/functions.h"
#include "disturbance/disturbance.h"
#include "mdp_net.h"

using namespace std;
using namespace geometry_utils;


namespace mdp_planner
{
  class MDP
  {

  public:

    typedef boost::shared_ptr<MDP> Ptr;
    typedef boost::shared_ptr<const MDP> ConstPtr;
    double *states_fpt_final;

  protected:

    utils::Parameters::Ptr  	pParams;
    MDP_Net::Ptr 		pNet;		//grid2d environment
    Disturbance::Ptr		pDisturb;
    mdp_state_t *start_state, *goal_state;		//spawned stand-alone start/goal states

    string iter_method;		//value vs. policy vs LP, etc
    uint num_iters;		//num of value iterations

    double action_cost;
    bool use_last_frame;	//use_last_frame follows standard VI, values updated from last iteration
    double obst_penalty; 	//obstacle penalty
    double epsilon;
  public:

    MDP(const utils::Parameters::Ptr&, const MDP_Net::Ptr&, const Disturbance::Ptr&);
    virtual ~MDP();

    //accessors
    inline MDP_Net::Ptr getpNet(void){ return pNet; }
    virtual void loadParams(void);

    //fill value & type for single state, if Starts or Goals, fill spawn_parent
    virtual void fillTypeValue(mdp_state_t* _state, type_t _type, double _value);
    virtual void fillTypeValueVec(vector<mdp_state_t*>& _states, type_t _type, vector<double>& _values);

    //if succ states are obstacles/walls, put a NEG penalty on such obst states
    virtual double addSuccessorValue(mdp_state_t* s,
                                     action_t succ_a,
                                     double percent);

    //get MDP P(s'|s, a, d) by computing discrete transition s->probs
    virtual void getTransitionModel(mdp_state_t* s,
                                    action_t act,
                                    const Vec2& v_disturb);

    //get MDP P(s'|s, a, d) by computing discrete transition s->post_probs
    virtual void getPostTransitionModel(mdp_state_t* s,
                                    action_t act,
                                    const Vec2& v_disturb);

    // fill in the local transition probability (size: NUM_ACTIONS = 9) based on the optimal action at every state.
    virtual void optimalActionTransitionDistribution(vector<mdp_state_t*>& _states);

    virtual void actionTransitionDistribution(vector<mdp_state_t*>& _states);

    virtual void optimalActionTransitionDistributionLocal(vector<mdp_state_t*>& _states, int startstate=0, int finishstate=30);

    virtual void optimalActionTransitionDistributionLocalStates(vector<mdp_state_t*>& _states, vector<int> statestoupdate);

    //get MDP Q(s, a) value by expectation over transition probs
    virtual double getQvalue(mdp_state_t* s,
                             action_t act);

    virtual double getQvalueQL(mdp_state_t* s,
                             action_t act);
    virtual double getQvalueQLOld(mdp_state_t* s,
                             action_t act);

    // choose the best value/action/time etc with argmax operation
    virtual void updateStateRecords(mdp_state_t* s);

    virtual void updateStateRecordsQLearning(mdp_state_t* s);

    // main iteration method
    virtual void iterations(void);

    virtual double prioritizedSweeping(vector<mdp_state_t*>&);



    // propagation direction can either be "forward" or "backward"
    virtual double valueIteration(vector<mdp_state_t*>&, string direction="forward");

    virtual double valueIterationPS(vector<mdp_state_t*>&);
    virtual double valueIterationPSD(vector<mdp_state_t*>&);

    // propagation direction can either be "forward" or "backward"
    virtual double valueIterationMFPT(vector<mdp_state_t*>&, int iteration_num, string direction="forward");

    // propagation direction can either be "forward" or "backward"
    virtual double valueIterationMFPTD(vector<mdp_state_t*>&, int iteration_num, string direction="forward");

    // propagation direction can either be "forward" or "backward"
    virtual double valueIterationMFPTD2(vector<mdp_state_t*>&, int iteration_num, string direction="forward");

    // propagation direction can either be "forward" or "backward"
    virtual void valueIterationMFPTSub(vector<mdp_state_t*>&, string direction="forward");

    virtual void valueIterationMFPTDSub(vector<mdp_state_t*>&, string direction="forward");

    virtual void valueIterationMFPTDSubH2(vector<mdp_state_t*>&, string direction="forward");

    // propagation direction can either be "forward" or "backward"
    virtual double valueIterationLocal(vector<mdp_state_t*>&, string direction="forward", int startstate=0, int finishstate=30);

    virtual double valueIterationLocalStates(vector<mdp_state_t*>&, vector<int>, string direction="forward");

    // faster convergence for the case of replanning??
    virtual void policyIteration(vector<mdp_state_t*>&);

    virtual void policyIterationPS(vector<mdp_state_t*>&);

    // prioritized sweeping
    virtual void policyIterationMFPT(vector<mdp_state_t*>&);

    // LP
    virtual void policyIterationLP(vector<mdp_state_t*>&);


    // QLearning, DYNA, MFPT-QL, DYNA MFPT
    virtual double QLearning(vector<mdp_state_t*>&);
    virtual double MFPT_QL(vector<mdp_state_t*>&);
    virtual double DYNA(vector<mdp_state_t*>&);
    virtual double MFPT_DYNA(vector<mdp_state_t*>&);
    virtual double DYNA_PS(vector<mdp_state_t*>&);

    static bool pairCompare(const std::pair<int, mdp_state_t*>& firstElem, const std::pair<int, mdp_state_t*>& secondElem);

    //clean/reset states
    virtual void cleanStates(void);

    double getObstPenalty() {
      return obst_penalty;
    }

  };

}


#endif
