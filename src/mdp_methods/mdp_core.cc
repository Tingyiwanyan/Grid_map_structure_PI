#include "geometry_utils/Vector4.h"
#include "mdp_methods/mdp_core.h"
#include "method_manager/method_manager.h"
#include "mdp_methods/ssp.h"
#include "method_manager/method_manager.h"
#include "animation/animation.h"
#include <map>
#include <fstream>
#include <queue>

using namespace mdp_planner;
using namespace std::chrono;

int it_cnt = 0;
int num_states;
vector<int>stateid;

vector<double> new_mfpt;
vector<double> old_mfpt;
int d_count = 1;


MDP::MDP(const utils::Parameters::Ptr& pParams, const MDP_Net::Ptr& pNet, const Disturbance::Ptr& pDisturb){

    //assert(pParams);
    //assert(pNet);
    //assert(pDisturb);
    this->pParams  = pParams;
    this->pNet  = pNet;
    this->pDisturb = pDisturb;

    loadParams();

    start_state = NULL;
    goal_state = NULL;

    num_states = pParams->getParamNode()["environment"]["grids"]["num_rows"].as<int>() * pParams->getParamNode()["environment"]["grids"]["num_cols"].as<int>();
    stateid.resize(num_states);
    new_mfpt.resize(num_states);
    old_mfpt.resize(num_states);

}


MDP::~MDP(){

    //if not null pointers, delete them
    if(start_state) delete start_state;
    if(goal_state) delete goal_state;
    // pNet is smart pointer and handles deletion itself

    cout<<"\t mdp_methods: destroyed start & goal states."<<endl;

}




void
MDP::loadParams(void){

    num_iters      = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
    iter_method    = pParams->getParamNode()["mdp_methods"]["iteration_method"].as<string>();
    action_cost    = pParams->getParamNode()["mdp_methods"]["action_cost"].as<double>();
    obst_penalty   = pParams->getParamNode()["mdp_methods"]["obst_penalty"].as<double>();
    use_last_frame = pParams->getParamNode()["mdp_methods"]["use_last_frame"].as<bool>();

    epsilon = pParams->getParamNode()["mdp_methods"]["epsilon"].as<double>();

}


void
MDP::fillTypeValue(mdp_state_t* _state, type_t _type,  double _value){

    mdp_state_t* s=_state;
    if(s->spawn_parent)
        s=s->spawn_parent;
    s->type = _type;
    s->optimal_value = _value;
    s->last_optimal_value = _value;

}


void
MDP::fillTypeValueVec(vector<mdp_state_t*>& _states, type_t _type, vector<double>& _values){

    assert(_states.size()==_values.size());
    for(uint i=0; i<_states.size(); i++)
        fillTypeValue(_states[i], _type, _values[i]);

}


double
MDP::addSuccessorValue(mdp_state_t* s, action_t succ_a, double percent){

    double value=0;

    if(s->successors[succ_a] != s){
        //Can be based on either last frame (standard way in textbook) or immediate frame (faster)
        value += use_last_frame ? percent * s->successors[succ_a]->last_optimal_value
                                : percent * s->successors[succ_a]->optimal_value;
    }
    else
        value += percent * obst_penalty;
    //if obstacles, or walls such that robot cannot move forward but return to current state, put a penalty on it

    return value;

}

void
MDP::getTransitionModel(mdp_state_t* s, action_t act, const Vec2& v_disturb)
{
    assert(s->successors[act]!=s);

    double v_max = pParams->getParamNode()["vehicle_model"]["v_max"].as<double>();
    double sigma_a = pParams->getParamNode()["vehicle_model"]["gaussian_sigma"].as<double>();
    double sigma_d = pParams->getParamNode()["disturbance"]["gaussian_sigma"].as<double>();
    double sigma = sqrt(sigma_a*sigma_a + sigma_d*sigma_d);
    bool back_transition = pParams->getParamNode()["mdp_methods"]["transition_model"]["back_transition"].as<bool>();

    //get the unit applied vec for action a
    Vec2 v_app = mdp_state_t::getActionVector(act);
    v_app = v_max*v_app;
    Vec2 v_net = v_app + v_disturb;

    //each prob is associated with a action/transition link
    s->probs.resize(NUM_ACTIONS, 0);

    double sum = 0;
    for(uint i=NORTH; i<NUM_ACTIONS; i++){
        if(s->successors[i] != s){
            Vec2 v_succ = pNet->cell_centers[s->successors[i]->id] -
                    pNet->cell_centers[s->id];
            double theta, cos_theta;
            if(v_net.norm() > EPSILON){
                cos_theta = v_net.dot(v_succ)/(v_net.norm()*v_succ.norm());
                theta = std::acos(std::min(std::max(cos_theta, -1.0), 1.0));
                //method1: using gaussian pdf to emulate gaussian distribution on a polar coordinate
                s->probs[i] = utils::gaussian_pdf(theta, 0, sigma);
                //method2: using cosine curve to emulate gaussian like distribution with p\in[0, 2]
                //s->probs[i] = (cos_theta+1);
            }
            else{
                theta = 0; sigma = 1e+2;
                //large sigma generates pretty much like "uniform" distriubtion
                s->probs[i] = utils::gaussian_pdf(theta, 0, sigma);
            }
            //if only forward transitions are possible, set reverse transitions 0
            if(!back_transition && cos_theta < 0){
                s->probs[i] = 0;
            }
        }
        else
            s->probs[i] = 0;

        sum += s->probs[i];
    }

    //normalize to prob
    for(uint i=NORTH; i<NUM_ACTIONS; i++)
        s->probs[i] = (sum<EPSILON) ? 0 : s->probs[i]/sum;

    //cout<<s->probs<<endl;
}

void
MDP::getPostTransitionModel(mdp_state_t* s, action_t act, const Vec2& v_disturb)
{
    //  assert(s->successors[act]!=s);

    double v_max = pParams->getParamNode()["vehicle_model"]["v_max"].as<double>();
    double sigma_a = pParams->getParamNode()["vehicle_model"]["gaussian_sigma"].as<double>();
    double sigma_d = pParams->getParamNode()["disturbance"]["gaussian_sigma"].as<double>();
    double sigma = sqrt(sigma_a*sigma_a + sigma_d*sigma_d);
    bool back_transition = pParams->getParamNode()["mdp_methods"]["transition_model"]["back_transition"].as<bool>();

    //get the unit applied vec for action a
    Vec2 v_app = mdp_state_t::getActionVector(act);
    v_app = v_max*v_app;
    Vec2 v_net = v_app + v_disturb;

    //each prob is associated with a action/transition link
    s->post_probs.resize(NUM_ACTIONS, 0);

    double sum = 0;
    for(uint i=NORTH; i<NUM_ACTIONS; i++){
        if(s->successors[i] != s){
            Vec2 v_succ = pNet->cell_centers[s->successors[i]->id] -
                    pNet->cell_centers[s->id];
            double theta, cos_theta;
            if(v_net.norm() > EPSILON){
                cos_theta = v_net.dot(v_succ)/(v_net.norm()*v_succ.norm());
                theta = std::acos(std::min(std::max(cos_theta, -1.0), 1.0));
                //method1: using gaussian pdf to emulate gaussian distribution on a polar coordinate
                s->post_probs[i] = utils::gaussian_pdf(theta, 0, sigma);
                //method2: using cosine curve to emulate gaussian like distribution with p\in[0, 2]
                //s->probs[i] = (cos_theta+1);
            }
            else{
                theta = 0; sigma = 1e+2;
                //large sigma generates pretty much like "uniform" distriubtion
                s->post_probs[i] = utils::gaussian_pdf(theta, 0, sigma);
            }
            //if only forward transitions are possible, set reverse transitions 0
            if(!back_transition && cos_theta < 0){
                s->post_probs[i] = 0;
            }
        }
        else
            s->post_probs[i] = 0;

        sum += s->post_probs[i];
    }

    //normalize to prob
    for(uint i=NORTH; i<NUM_ACTIONS; i++)
        s->post_probs[i] = (sum<EPSILON) ? 0 : s->post_probs[i]/sum;

    //cout<<s->post_probs<<endl;
}



double
MDP::getQvalue(mdp_state_t* s, action_t act){

    s->q_values[act] = 0;

    for(uint i=NORTH; i<NUM_ACTIONS; i++){
        if(s->successors[i] != s){
            s->q_values[act] += addSuccessorValue(s, action_t(i), s->probs[i]);
        }
    }

    //here uniform action cost, can also be put in addSuccessorValue with percentage
    s->q_values[act] -= action_cost;

    return s->q_values[act];

}


double
MDP::getQvalueQLOld(mdp_state_t* s, action_t act){

    //s->q_values[act] = 0;

    double value=0;
    for(uint i=NORTH; i<NUM_ACTIONS; i++){

        if(s->successors[i] != s){

            mdp_state_t* next_st = (s->successors[action_t(i)]);
            //mdp_state_t* next_st = (s->successors[action_t(act)]);

            double v = -INF;
            for (int j = NORTH; j < NUM_ACTIONS; j++)
            {
                if (next_st->q_values[(action_t)j] > v)
                    v = next_st->q_values[(action_t)j];
            }
            next_st->optimal_value = v;

            value += s->probs[i] * v;
            //value += v;
        }
        else
            value += s->probs[i] * obst_penalty;

    }
    s->q_values[act] += value;
    //here uniform action cost, can also be put in addSuccessorValue with percentage
    s->q_values[act] -= action_cost;

    return s->q_values[act];

}

double
MDP::getQvalueQL(mdp_state_t* s, action_t act){

    //s->q_values[act] = 0;

    double value=0;
    for(uint i=NORTH; i<NUM_ACTIONS; i++){

        if(s->successors[i] != s){

            mdp_state_t* next_st = (s->successors[action_t(i)]);

            double imm_rew = (next_st->type == GOAL) ? 100 : action_cost;
            value += s->probs[i] * (imm_rew + 0.99* next_st->optimal_value);

        }
        else
            value += s->probs[i] * obst_penalty;

    }
    s->q_values[act] += value;

    return s->q_values[act];

}


void
MDP::updateStateRecords(mdp_state_t* s){

    int num_states_iter = 1;
    Vec2 v_disturb = pDisturb->getVec(s->id);

    //only body and start states need update, obstacles and goals no need do so
    //also applies on spawned start (get equal values of spawned and parent, with only the parent will be used though)
    if(s->type == BODY || s->type == START){
        std::fill(s->q_values.begin(), s->q_values.end(), 0);
        double max_val =  -INF; //s->optimal_value;
        s->q_values[ZERO] = -INF;
        for(uint j=NORTH; j<NUM_ACTIONS; j++){
            action_t act = (action_t)j;
            for(int i=1; i<=num_states_iter; i++){
                if(s->successors[act] != s) //the act should be legal
                {
                    getTransitionModel(s, act, v_disturb);
                    double direct_action_value = getQvalue(s, act);
                    max_val = (direct_action_value > max_val) ? direct_action_value : max_val;
                }
            }
        }


        //fill the max value as current value
        s->optimal_value = max_val;

        //cout<<"maxval: "<<max_val<<endl;

        //set current actions
        std::fill(s->actions.begin(), s->actions.end(), false);


        //then for all actions that has value/effect of == max_value, set true
        for(uint j=0; j<s->actions.size(); j++){
            if(fabs(s->q_values[j] - max_val) < EPSILON){
                s->actions[j] = true;
                s->optimal_action = (action_t)j;
            }
        }//for
    }//if

}

void
MDP::updateStateRecordsQLearning(mdp_state_t* s){

    int num_states_iter = 1;
    double discount_factor = pParams->getParamNode()["mdp_methods"]["discount_factor"].as<double>();


    //only body and start states need update, obstacles and goals no need do so
    if(s->type == BODY || s->type == START){
        std::fill(s->q_values.begin(), s->q_values.end(), 0);
        double max_val =  -INF; //s->optimal_value;
        s->q_values[ZERO] = -INF;
        for(uint j=NORTH; j<NUM_ACTIONS; j++){
            action_t act = (action_t)j;
            for(int i=1; i<=num_states_iter; i++){
                if(s->successors[act] != s) //the act should be legal
                {
                    //double direct_action_value = getQvalueQL(s, act);

                    mdp_state_t* next_st = (s->successors[act]);
                    double imm_rew = (next_st->type == GOAL) ? 100 : action_cost;

                    double direct_action_value = imm_rew + (discount_factor * next_st->optimal_value);
                    s->q_values[act] = direct_action_value;
                    max_val = (direct_action_value > max_val) ? direct_action_value : max_val;
                }
            }
        }

        //fill the max value as current value
        s->optimal_value = max_val;

    }//if

}


void
MDP::iterations(void){
    string config_file("../configs/config.yaml");
    pParams = utils::Parameters::Ptr(new utils::Parameters(config_file));

    cout<<"need to finish "<<num_iters<<" iterations."<<endl;
    cout<<"iteration progress: "<<std::flush;
    if (iter_method.compare("VI") == 0)
    {
        uint i;
        for ( i = 0; i < 1000; i++)
        {
            //cout << i << " " << std::flush;
            double d = valueIteration(pNet->mdp_states);
            cout << d << endl;

            if(d < 0.1)
                break;

        }
        cout << "Closed: " << i << endl;
    }
    else if (iter_method.compare("MFPT-VI") == 0)
    {
        uint i;
        num_iters = pParams->getParamNode()["environment"]["grids"]["num_iterations"].as<unsigned int>();
        for ( i = 0; i < num_iters; i++)
        {
            //cout << i << " " << std::flush;
            double d = valueIterationMFPT(pNet->mdp_states,i);
            cout << d << endl;

            if(d < 0.1)
                break;

        }
        cout << "Closed: " << i << endl;
    }
    else if(iter_method.compare("VI-PS") == 0){
        valueIterationPS(pNet->mdp_states);
    }
    else if(iter_method.compare("D-VI-PS") == 0){
        valueIterationPSD(pNet->mdp_states);
    }
    else if (iter_method.compare("D-MFPT-VI") == 0)
    {
        uint i;
        for ( i = 0; i < 1000; i++)
        {
            double d = valueIterationMFPTD(pNet->mdp_states,i);
            cout << d << endl;

            if(d < 0.1)
                break;

        }
        cout << "Closed: " << i << endl;
    }
    else if (iter_method.compare("D2-MFPT-VI") == 0)
    {
        uint i;

        for(int k = 0; k < num_states; k++){
            stateid[k] = k;
        }

        for ( i = 0; i < 1000; i++)
        {
            double d = valueIterationMFPTD2(pNet->mdp_states,i);
            cout << d << endl;

            if(d < 0.1)
                break;

        }
        cout << "Closed: " << i << endl;
    }
    else if (iter_method.compare("PI") == 0)
    {
        policyIteration(pNet->mdp_states);
    }
    else if(iter_method.compare("PI-PS") == 0){
        policyIterationPS(pNet->mdp_states);
    }
    else if(iter_method.compare("PI-LP") == 0){
        policyIterationLP(pNet->mdp_states);
    }
    else if (iter_method.compare("MFPT-PI") == 0)
    {
        policyIterationMFPT(pNet->mdp_states);
    }
    else if (iter_method.compare("MFPT-VI-Sub") == 0)
    {
        valueIterationMFPTSub(pNet->mdp_states);
    }
    else if (iter_method.compare("D-MFPT-VI-H1-Sub") == 0)
    {
        valueIterationMFPTDSub(pNet->mdp_states);
    }
    else if (iter_method.compare("D-MFPT-VI-H2-Sub") == 0)
    {
        valueIterationMFPTDSubH2(pNet->mdp_states);
    }
    else if (iter_method.compare("qlearning") == 0)
    {
        QLearning(pNet->mdp_states);
    }
    else if (iter_method.compare("MFPTQL") == 0)
    {
        MFPT_QL(pNet->mdp_states);
    }
    else if (iter_method.compare("DYNA") == 0)
    {
        DYNA(pNet->mdp_states);
    }
    else if (iter_method.compare("MFPTDYNA") == 0)
    {
        MFPT_DYNA(pNet->mdp_states);
    }
    else if (iter_method.compare("DYNAPS") == 0)
    {
        DYNA_PS(pNet->mdp_states);
    }
    else
    {
        cerr << "Could not find a solution method." << endl;
        exit(0);
    }

    cout<<endl;
}

void
MDP::optimalActionTransitionDistribution(vector<mdp_state_t*>& _states) {
    for (vector<mdp_state_t*>::iterator itr = _states.begin(); itr != _states.end(); itr++) {
        mdp_state_t* s = *itr;
        Vec2 v_disturb = pDisturb->getVec(s->id);
        getPostTransitionModel(s, s->optimal_action, v_disturb);
        //    std::cout << "s " << s->id << std::endl;
        //    std::cout << "s->optimal_action " << s->optimal_action << std::endl;
        //    for (const auto &d : s->post_probs) {
        //      std::cout << d << " ";
        //    }
        //    std::cout << std::endl;
    }
}


void
MDP::actionTransitionDistribution(vector<mdp_state_t*>& _states) {
    for (vector<mdp_state_t*>::iterator itr = _states.begin(); itr != _states.end(); itr++) {
        mdp_state_t* s = *itr;
        Vec2 v_disturb = pDisturb->getVec(s->id);
        //getPostTransitionModel(s, s->optimal_action, v_disturb);
        for(uint j=NORTH; j<NUM_ACTIONS; j++){
            action_t act = (action_t)j;
            if(s->successors[act] != s) //the act should be legal
            {
                getPostTransitionModel(s, act, v_disturb);
                //double direct_action_value = getQvalue(s, act);
                //max_val = (direct_action_value > max_val) ? direct_action_value : max_val;
            }
        }
    }
}

void
MDP::optimalActionTransitionDistributionLocal(vector<mdp_state_t*>& _states, int startstate, int finishstate) {
    for (vector<mdp_state_t*>::iterator itr = _states.begin(); itr != _states.end(); itr++) {
        if((*itr)->id >= startstate && (*itr)->id <= finishstate){
            mdp_state_t* s = *itr;
            Vec2 v_disturb = pDisturb->getVec(s->id);
            getPostTransitionModel(s, s->optimal_action, v_disturb);
            //    std::cout << "s " << s->id << std::endl;
            //    std::cout << "s->optimal_action " << s->optimal_action << std::endl;
            //    for (const auto &d : s->post_probs) {
            //      std::cout << d << " ";
            //    }
            //    std::cout << std::endl;
        }
    }
}

void
MDP::optimalActionTransitionDistributionLocalStates(vector<mdp_state_t*>& _states, vector<int> statestoupdate) {
    for (vector<int>::iterator states_itr = statestoupdate.begin(); states_itr != statestoupdate.end(); states_itr++) {
        for (vector<mdp_state_t*>::iterator itr = _states.begin(); itr != _states.end(); itr++) {
            if((*itr)->id == (*states_itr)){
                mdp_state_t* s = *itr;
                Vec2 v_disturb = pDisturb->getVec(s->id);
                getPostTransitionModel(s, s->optimal_action, v_disturb);
            }
        }
    }
}

double
MDP::valueIteration(vector<mdp_state_t*>& _states, string direction){

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    if(direction=="forward"){
        for(vector<mdp_state_t*>::iterator itr=_states.begin(); itr!=_states.end();itr++){
            if((*itr)->type==BODY || (*itr)->type==START)
                updateStateRecords(*itr);
            //else if((*itr)->type==START && (*itr)->spawn_parent)
            //  updateStateRecords((*itr)->spawn_parent);
            else
                continue; //ignore goals or obstacles;
        }//for
    }
    else if(direction=="backward"){
        for(vector<mdp_state_t*>::reverse_iterator ritr=_states.rbegin(); ritr!=_states.rend(); ritr++){
            if((*ritr)->type==BODY || (*ritr)->type==START)
                updateStateRecords(*ritr);
            //else if((*ritr)->type==START && (*ritr)->spawn_parent)
            //  updateStateRecords((*ritr)->spawn_parent);
            else
                continue; //ignore goals or obstacles;
        }//for
    }
    else{
        cerr<<"Propagation direction!!"<<endl;
        assert(0);
    }


    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
    cout << "\nTime Taken Iteration VI: " << duration/1000000.0 << " seconds" << endl;

    //compare epsilon value and snapshot, for next iteration
    double max_delta=0;
    for(uint i=0; i<_states.size(); i++){
        mdp_state_t* s=_states[i];
        // for those spawn states
        if(s->spawn_parent){
            s=s->spawn_parent;
        }
        double delta = fabs(s->optimal_value - s->last_optimal_value);
        max_delta = (delta>max_delta)? delta: max_delta;
        s->last_optimal_value = s->optimal_value;
        s->last_optimal_action = s->optimal_action;
    }

    //cout<<"max_delta: "<<max_delta<<endl;

    return max_delta;

}

double
MDP::valueIterationPS(vector<mdp_state_t*>& _states){

    vector<pair<double,mdp_state_t*>> _sorted_states;
    vector<pair<double,mdp_state_t*>> _new_sorted_states;
    for(vector<mdp_state_t*>::iterator itr=_states.begin(); itr!=_states.end();itr++){
        if((*itr)->type==BODY || (*itr)->type==START)
            _sorted_states.push_back(make_pair(0, *itr));
        else
            continue; //ignore goals or obstacles;
    }//for

    string direction = "forward";

    int i = 0;
    double max_delta=0;
    do{
        high_resolution_clock::time_point t1 = high_resolution_clock::now();


        if(direction=="forward"){
            for(pair<double,mdp_state_t*> itr : _sorted_states){
                if((itr.second)->type==BODY || (itr.second)->type==START){
                    double old_val = (itr.second)->optimal_value;
                    updateStateRecords(itr.second);
                    double diff = (itr.second)->optimal_value - old_val;
                    _new_sorted_states.push_back(make_pair(diff, itr.second));
                }
                else
                    continue; //ignore goals or obstacles;
            }//for
        }



        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        cout << "\nTime Taken Iteration VI-PS Bellman: " << duration/1000000.0 << " seconds" << endl;


        high_resolution_clock::time_point t11 = high_resolution_clock::now();

        std::sort(_new_sorted_states.begin(), _new_sorted_states.end(), [](pair<double,mdp_state_t*> &left, pair<double,mdp_state_t*> &right) {
            return left.first > right.first;
        });

        high_resolution_clock::time_point t22 = high_resolution_clock::now();
        auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
        cout << "\nTime Taken Iteration VI-PS Sorting: " << duration1/1000000.0 << " seconds" << endl;

        _sorted_states = _new_sorted_states;
        _new_sorted_states.clear();
        i++;


        //compare epsilon value and snapshot, for next iteration
        max_delta=0;
        for(uint i=0; i<_states.size(); i++){
            mdp_state_t* s=_states[i];
            // for those spawn states
            if(s->spawn_parent){
                s=s->spawn_parent;
            }
            double delta = fabs(s->optimal_value - s->last_optimal_value);
            max_delta = (delta>max_delta)? delta: max_delta;
            s->last_optimal_value = s->optimal_value;
            s->last_optimal_action = s->optimal_action;
        }
        //cout << "i: " << i << " , Delta: " << max_delta << endl;
        cout << max_delta << endl;
    }
    while(max_delta > 0.1);

    //cout<<"max_delta: "<<max_delta<<endl;
    cout << i << endl;
    return max_delta;

}

double
MDP::valueIterationPSD(vector<mdp_state_t*>& _states){

    vector<pair<double,mdp_state_t*>> _sorted_states;
    vector<pair<double,mdp_state_t*>> _new_sorted_states;
    vector<pair<int,mdp_state_t*>> _states_old_value;
    for(vector<mdp_state_t*>::iterator itr=_states.begin(); itr!=_states.end();itr++){
        if((*itr)->type==BODY || (*itr)->type==START)
            _sorted_states.push_back(make_pair(0, *itr));
        else
            continue; //ignore goals or obstacles;
    }//for

    string direction = "forward";

    int i = 0;
    double max_delta=0;
    int t=1;
    do{
        //cout << "t: " << i << ", cnt: " << _sorted_states.size() << endl;
        high_resolution_clock::time_point t1 = high_resolution_clock::now();


        if(direction=="forward"){
            int itr_cnt = -1;
            for(pair<double,mdp_state_t*> itr : _sorted_states){
                //cout << "ID: " << (itr.second)->id << endl;
                if((itr.second)->type==BODY || (itr.second)->type==START){
                    itr_cnt++;
                    if(t==1) { _states_old_value.push_back(make_pair((itr.second)->id, itr.second)); }
                    updateStateRecords(itr.second);
                    if(t == 2) {
                        double diff = (itr.second)->optimal_value - (_states_old_value[itr_cnt].second)->optimal_value;
                        _new_sorted_states.push_back(make_pair(diff, itr.second));
                    }
                }
                else
                    continue; //ignore goals or obstacles;
            }//for
        }



        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        cout << "\nTime Taken Iteration VI-PS D Bellman: " << duration/1000000.0 << " seconds" << endl;


        high_resolution_clock::time_point t11 = high_resolution_clock::now();

        std::sort(_new_sorted_states.begin(), _new_sorted_states.end(), [](pair<double,mdp_state_t*> &left, pair<double,mdp_state_t*> &right) {
            return left.first > right.first;
        });

        high_resolution_clock::time_point t22 = high_resolution_clock::now();
        auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
        cout << "\nTime Taken Iteration VI-PS D Sorting: " << duration1/1000000.0 << " seconds" << endl;

        if(t==2){
            _sorted_states = _new_sorted_states;
            _new_sorted_states.clear();
            t = 1;
            _states_old_value.clear();
        }
        else {
            t++;
        }
        i++;


        //compare epsilon value and snapshot, for next iteration
        max_delta=0;
        for(uint i=0; i<_states.size(); i++){
            mdp_state_t* s=_states[i];
            // for those spawn states
            if(s->spawn_parent){
                s=s->spawn_parent;
            }
            double delta = fabs(s->optimal_value - s->last_optimal_value);
            max_delta = (delta>max_delta)? delta: max_delta;
            s->last_optimal_value = s->optimal_value;
            s->last_optimal_action = s->optimal_action;
        }
        //cout << "i: " << i << " , Delta: " << max_delta << endl;
        cout << max_delta << endl;
    }
    while(max_delta > 0.1);

    //cout<<"max_delta: "<<max_delta<<endl;
    cout << "Closed: " << i << endl;
    return max_delta;

}

double
MDP::prioritizedSweeping(vector<mdp_state_t*>& _states){

    map<int, pair<char, int>> dict = {{1, make_pair('a', 12)}, {2, make_pair('b', 9)}, {5, make_pair('c', 7)}, {4, make_pair('d', 10)},};
    struct cmp {
        bool operator()(const pair<int, pair<char, int>> &a, const pair<int, pair<char, int>> &b) {
            return a.first < b.first;
        };
    };

    struct cmp1 {
        bool operator()(const pair<double, int> &a, const pair<double, int> &b) {
            return a.first > b.first;
        };
    };

    vector<Transform2> tf2_starts, tf2_goals;
    tf2_starts.push_back(Transform2(0, -17, 0.1));
    tf2_goals.push_back(Transform2(0, 17, 0.5));

    //priority_queue<pair<int, pair<char, int>>, vector<pair<int, pair<char, int>>>, cmp> pq;
    set<pair<double, int>, cmp1> pq;
    //pq.push({3, make_pair('e', 12)});

    mdp_state_t* s0 = pNet->getState(tf2_starts[0].translation);
    cout << s0->id << ", " << s0->q_values[(action_t)0];

    mdp_state_t* s = s0;
    double old_val = s->optimal_value;
    if((s)->type==BODY || (s)->type==START)
        updateStateRecords(s);
    double delta = s->optimal_value - old_val;
    cout << " Delta: " << delta << endl;
    cout << " Old: " << old_val << ", New: " << s->optimal_value << endl;
    pq.insert(make_pair(delta,s->id));
    //pq.insert(make_pair(-2,100));

    while (!pq.empty()) {
        pair<double, int> top = *(pq.begin());
        cout << top.first << " : " << top.second << endl;
        mdp_state_t* s1 = _states[top.second];
        for(mdp_state_t* s_pred : s1->predecessors){
            cout << s_pred->id << " ";
            double old_val1 = s_pred->optimal_value;
            if((s_pred)->type==BODY || (s_pred)->type==START)
                updateStateRecords(s_pred);
            pq.insert(make_pair(s_pred->optimal_value - old_val1,s_pred->id));
        }
        cout << endl;
        pq.erase(top);
    }

    return 1.0;

}

double
MDP::valueIterationMFPT(vector<mdp_state_t*>& _states, int iteration_num, string direction){
    std::cout<<"Im here in MFPT value iteration"<<std::endl;
    vector<mdp_state_t*> _prioritizedstates;
    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    std::cout<<"start position"<<std::endl<<vec[0]<<"  "<<vec[1]<<"  "<<vec[2]<<std::endl;
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    if(direction=="forward"){

        if(iteration_num >= 0 && iteration_num < 10000 && iteration_num % 1 == 0){
            SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

            high_resolution_clock::time_point t1 = high_resolution_clock::now();

            _prioritizedstates.clear();
            optimalActionTransitionDistribution(_states);
            pSSP->initTransMatrix();
            double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);

            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>( t2 - t1 ).count();
            cout << "\nTime Taken Iteration MFPT-VI FPT: " << duration/1000000.0 << " seconds" << endl;
            //glutDisplayFunc(&display);

            //render_mdp::findPlaneWrapper();

            //glutMainLoop();

            vector<int>stateid(num_states);
            it_cnt = 0;
            for (pair<double,int> it : pSSP->sweepedStates) {
                stateid[it_cnt] = it.second;
                it_cnt++;
            }
            pSSP->sweepedStates.clear();

            high_resolution_clock::time_point t11 = high_resolution_clock::now();

            for(int state_cnt = 0; state_cnt < it_cnt; state_cnt++){
                if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                    updateStateRecords(_states[stateid[state_cnt]]);
                //else if((*itr)->type==START && (*itr)->spawn_parent)
                //  updateStateRecords((*itr)->spawn_parent);
                else
                    continue; //ignore goals or obstacles;
            }//for

            high_resolution_clock::time_point t22 = high_resolution_clock::now();
            auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
            cout << "\nTime Taken Iteration MFPT-VI Bellman: " << duration1/1000000.0 << " seconds" << endl;
        }
        else{

            for(vector<mdp_state_t*>::iterator itr=_states.begin(); itr!=_states.end();itr++){
                if((*itr)->type==BODY || (*itr)->type==START)
                    updateStateRecords(*itr);
                //else if((*itr)->type==START && (*itr)->spawn_parent)
                //  updateStateRecords((*itr)->spawn_parent);
                else
                    continue; //ignore goals or obstacles;
            }//for
        }
    }
    else if(direction=="backward"){
        for(vector<mdp_state_t*>::reverse_iterator ritr=_states.rbegin(); ritr!=_states.rend(); ritr++){
            if((*ritr)->type==BODY || (*ritr)->type==START)
                updateStateRecords(*ritr);
            //else if((*ritr)->type==START && (*ritr)->spawn_parent)
            //  updateStateRecords((*ritr)->spawn_parent);
            else
                continue; //ignore goals or obstacles;
        }//for
    }
    else{
        cerr<<"Propagation direction!!"<<endl;
        assert(0);
    }

    //compare epsilon value and snapshot, for next iteration
    double max_delta=0;
    for(uint i=0; i<_states.size(); i++){
        mdp_state_t* s=_states[i];
        // for those spawn states
        if(s->spawn_parent){
            s=s->spawn_parent;
        }
        double delta = fabs(s->optimal_value - s->last_optimal_value);
        max_delta = (delta>max_delta)? delta: max_delta;
        s->last_optimal_value = s->optimal_value;
        s->last_optimal_action = s->optimal_action;
    }

    //cout<<"max_delta: "<<max_delta<<endl;
    return max_delta;

}


double
MDP::valueIterationMFPTD(vector<mdp_state_t*>& _states, int iteration_num, string direction){

    vector<mdp_state_t*> _prioritizedstates;
    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    if(direction=="forward"){

        if(iteration_num >= 0 && iteration_num < 10000 && iteration_num % 3 == 0){
            SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

            high_resolution_clock::time_point t1 = high_resolution_clock::now();


            vector<double> new_mfpt(num_states);
            vector<double> old_mfpt(num_states);


            for (pair<double,int> it : pSSP->sweepedStates) {
                old_mfpt[it.second] = it.first;
            }


            _prioritizedstates.clear();
            optimalActionTransitionDistribution(_states);
            pSSP->initTransMatrix();
            pSSP->sweepedStates.clear();

            double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);

            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>( t2 - t1 ).count();
            cout << "\nTime Taken Iteration D-MFPT-VI FPT: " << duration/1000000.0 << " seconds" << endl;

            //int num_states = 2500;

            for (pair<double,int> it : pSSP->sweepedStates) {
                new_mfpt[it.second] = it.first;
            }

            vector<pair<double,int>> _new_sorted_states;
            for(int t = 0; t < _states.size(); t++){
                _new_sorted_states.push_back(make_pair(fabs(new_mfpt[t] - old_mfpt[t]), t));
            }
            std::sort(_new_sorted_states.begin(), _new_sorted_states.end(), [](pair<double,int> &left, pair<double,int> &right) {
                return left.first < right.first;
            });

            //          for(pair<double,int> ele : _new_sorted_states){
            //              cout << ele.second << " ";
            //          }
            //          cout << endl;

            //vector<int>stateid(num_states);
            int it_cnt = 0;
            for (pair<double,int> it : _new_sorted_states) {
                stateid[it_cnt] = it.second;
                it_cnt++;
            }
            _new_sorted_states.clear();


            high_resolution_clock::time_point t11 = high_resolution_clock::now();

            for(int state_cnt = 0; state_cnt < it_cnt; state_cnt++){
                if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                    updateStateRecords(_states[stateid[state_cnt]]);
                //else if((*itr)->type==START && (*itr)->spawn_parent)
                //  updateStateRecords((*itr)->spawn_parent);
                else
                    continue; //ignore goals or obstacles;
            }//for

            high_resolution_clock::time_point t22 = high_resolution_clock::now();
            auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
            cout << "\nTime Taken Iteration D-MFPT-VI Bellman: " << duration1/1000000.0 << " seconds" << endl;
        }
        else{

            for(int state_cnt = 0; state_cnt < _states.size(); state_cnt++){
                if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                    updateStateRecords(_states[stateid[state_cnt]]);
                //else if((*itr)->type==START && (*itr)->spawn_parent)
                //  updateStateRecords((*itr)->spawn_parent);
                else
                    continue; //ignore goals or obstacles;
            }//for
        }
    }
    else if(direction=="backward"){
        for(vector<mdp_state_t*>::reverse_iterator ritr=_states.rbegin(); ritr!=_states.rend(); ritr++){
            if((*ritr)->type==BODY || (*ritr)->type==START)
                updateStateRecords(*ritr);
            //else if((*ritr)->type==START && (*ritr)->spawn_parent)
            //  updateStateRecords((*ritr)->spawn_parent);
            else
                continue; //ignore goals or obstacles;
        }//for
    }
    else{
        cerr<<"Propagation direction!!"<<endl;
        assert(0);
    }

    //compare epsilon value and snapshot, for next iteration
    double max_delta=0;
    for(uint i=0; i<_states.size(); i++){
        mdp_state_t* s=_states[i];
        // for those spawn states
        if(s->spawn_parent){
            s=s->spawn_parent;
        }
        double delta = fabs(s->optimal_value - s->last_optimal_value);
        max_delta = (delta>max_delta)? delta: max_delta;
        s->last_optimal_value = s->optimal_value;
        s->last_optimal_action = s->optimal_action;
    }

    //cout<<"max_delta: "<<max_delta<<endl;
    return max_delta;

}

double
MDP::valueIterationMFPTD2(vector<mdp_state_t*>& _states, int iteration_num, string direction){

    vector<mdp_state_t*> _prioritizedstates;
    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    if(direction=="forward"){

        if(iteration_num >= 0 && iteration_num < 10000 && iteration_num % 2 == 0){
            SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

            high_resolution_clock::time_point t1 = high_resolution_clock::now();


            if(d_count == 1){
                for (pair<double,int> it : pSSP->sweepedStates) {
                    old_mfpt[it.second] = it.first;
                }
            }


            _prioritizedstates.clear();
            optimalActionTransitionDistribution(_states);
            pSSP->initTransMatrix();
            pSSP->sweepedStates.clear();

            double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);

            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>( t2 - t1 ).count();
            cout << "\nTime Taken Iteration D2-MFPT-VI FPT: " << duration/1000000.0 << " seconds" << endl;

            if(d_count == 2){
                for (pair<double,int> it : pSSP->sweepedStates) {
                    new_mfpt[it.second] = it.first;
                }

                vector<pair<double,int>> _new_sorted_states;
                for(int t = 0; t < _states.size(); t++){
                    _new_sorted_states.push_back(make_pair(fabs(new_mfpt[t] - old_mfpt[t]), t));
                }
                std::sort(_new_sorted_states.begin(), _new_sorted_states.end(), [](pair<double,int> &left, pair<double,int> &right) {
                    return left.first < right.first;
                });

                int it_cnt = 0;
                for (pair<double,int> it : _new_sorted_states) {
                    stateid[it_cnt] = it.second;
                    it_cnt++;
                }
                _new_sorted_states.clear();
                d_count = 1;
            }
            else {
                d_count++;
            }


            high_resolution_clock::time_point t11 = high_resolution_clock::now();

            for(int state_cnt = 0; state_cnt < _states.size(); state_cnt++){
                if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                    updateStateRecords(_states[stateid[state_cnt]]);
                //else if((*itr)->type==START && (*itr)->spawn_parent)
                //  updateStateRecords((*itr)->spawn_parent);
                else
                    continue; //ignore goals or obstacles;
            }//for

            high_resolution_clock::time_point t22 = high_resolution_clock::now();
            auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
            cout << "\nTime Taken Iteration D2-MFPT-VI Bellman: " << duration1/1000000.0 << " seconds" << endl;
        }
        else{

            for(int state_cnt = 0; state_cnt < _states.size(); state_cnt++){
                if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                    updateStateRecords(_states[stateid[state_cnt]]);
                //else if((*itr)->type==START && (*itr)->spawn_parent)
                //  updateStateRecords((*itr)->spawn_parent);
                else
                    continue; //ignore goals or obstacles;
            }//for
        }
    }
    else if(direction=="backward"){
        for(vector<mdp_state_t*>::reverse_iterator ritr=_states.rbegin(); ritr!=_states.rend(); ritr++){
            if((*ritr)->type==BODY || (*ritr)->type==START)
                updateStateRecords(*ritr);
            //else if((*ritr)->type==START && (*ritr)->spawn_parent)
            //  updateStateRecords((*ritr)->spawn_parent);
            else
                continue; //ignore goals or obstacles;
        }//for
    }
    else{
        cerr<<"Propagation direction!!"<<endl;
        assert(0);
    }

    //compare epsilon value and snapshot, for next iteration
    double max_delta=0;
    for(uint i=0; i<_states.size(); i++){
        mdp_state_t* s=_states[i];
        // for those spawn states
        if(s->spawn_parent){
            s=s->spawn_parent;
        }
        double delta = fabs(s->optimal_value - s->last_optimal_value);
        max_delta = (delta>max_delta)? delta: max_delta;
        s->last_optimal_value = s->optimal_value;
        s->last_optimal_action = s->optimal_action;
    }

    //cout<<"max_delta: "<<max_delta<<endl;
    return max_delta;

}


void
MDP::valueIterationMFPTSub(vector<mdp_state_t*>& _states, string direction){

    vector<mdp_state_t*> _prioritizedstates;
    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    int partition_num = pParams->getParamNode()["environment"]["partitions"]["num_partitions"].as<int>();;

    for(int part_int = 0; part_int < partition_num; part_int++){

        int loop_cnt_part = -1;

        int start_state, end_state;



        while (true) {

            loop_cnt_part++;

            if(direction=="forward"){

                if(loop_cnt_part >= 0 && loop_cnt_part < 10000 && loop_cnt_part % 2 == 0 && loop_cnt_part <= 6){
                    SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));
                    pSSP->loopind = part_int + 1;

                    high_resolution_clock::time_point t1 = high_resolution_clock::now();

                    _prioritizedstates.clear();
                    optimalActionTransitionDistribution(_states);
                    pSSP->initTransMatrix();
                    double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);

                    high_resolution_clock::time_point t2 = high_resolution_clock::now();
                    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
                    cout << "\nTime Taken Iteration MFPT-VI-Sub FPT: " << duration/1000000.0 << " seconds" << endl;

                    it_cnt = 0;

                    for (pair<double,int> it : pSSP->sweepedStates) {
                        stateid[it_cnt] = it.second;
                        it_cnt++;
                    }
                    pSSP->sweepedStates.clear();

                    start_state = part_int*it_cnt/partition_num;
                    //start_state = 0;
                    end_state = (part_int == (partition_num - 1)) ? (((part_int + 1) * it_cnt)/partition_num) : ((((part_int+1)*it_cnt)/partition_num) + it_cnt/250);
                    //end_state = (((part_int + 1) * it_cnt)/partition_num);
                    cout << "Start: " << start_state << " End: " << end_state << endl;

                    high_resolution_clock::time_point t11 = high_resolution_clock::now();

                    for(int state_cnt = start_state; state_cnt < end_state; state_cnt++){
                        if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                            updateStateRecords(_states[stateid[state_cnt]]);
                        //else if((*itr)->type==START && (*itr)->spawn_parent)
                        //  updateStateRecords((*itr)->spawn_parent);
                        else
                            continue; //ignore goals or obstacles;
                    }//for

                    high_resolution_clock::time_point t22 = high_resolution_clock::now();
                    auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
                    cout << "\nTime Taken Iteration MFPT-VI-Sub Bellman: " << duration1/1000000.0 << " seconds" << endl;
                }
                else{

                    //cout << it_cnt << endl;

                    for(int state_cnt = start_state; state_cnt < end_state; state_cnt++){
                        if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                            updateStateRecords(_states[stateid[state_cnt]]);
                        //else if((*itr)->type==START && (*itr)->spawn_parent)
                        //  updateStateRecords((*itr)->spawn_parent);
                        else
                            continue; //ignore goals or obstacles;
                    }//for
                }
            }
            else if(direction=="backward"){
                for(vector<mdp_state_t*>::reverse_iterator ritr=_states.rbegin(); ritr!=_states.rend(); ritr++){
                    if((*ritr)->type==BODY || (*ritr)->type==START)
                        updateStateRecords(*ritr);
                    //else if((*ritr)->type==START && (*ritr)->spawn_parent)
                    //  updateStateRecords((*ritr)->spawn_parent);
                    else
                        continue; //ignore goals or obstacles;
                }//for
            }
            else{
                cerr<<"Propagation direction!!"<<endl;
                assert(0);
            }

            //compare epsilon value and snapshot, for next iteration
            double max_delta=0;
            //for(uint i=0; i<_states.size(); i++){
            for(int state_cnt = start_state; state_cnt < end_state; state_cnt++){
                mdp_state_t* s=_states[stateid[state_cnt]];
                // for those spawn states
                if(s->spawn_parent){
                    s=s->spawn_parent;
                }
                double delta = fabs(s->optimal_value - s->last_optimal_value);
                max_delta = (delta>max_delta)? delta: max_delta;
                s->last_optimal_value = s->optimal_value;
                s->last_optimal_action = s->optimal_action;
            }

            cout << max_delta << endl;
            if(max_delta < 0.1)
                break;

        }

        cout << "Closed Partition: " << part_int+1 << " Loop: " << loop_cnt_part << endl;

    }

}

void
MDP::valueIterationMFPTDSub(vector<mdp_state_t*>& _states, string direction){

    vector<mdp_state_t*> _prioritizedstates;
    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    int partition_num = pParams->getParamNode()["environment"]["partitions"]["num_partitions"].as<int>();

    for(int part_int = 0; part_int < partition_num; part_int++){

        int loop_cnt_part = -1;

        int start_state, end_state;



        while (true) {

            loop_cnt_part++;

            if(direction=="forward"){

                if(loop_cnt_part >= 0 && loop_cnt_part < 10000 && loop_cnt_part % 2 == 0 && loop_cnt_part <= 6){
                    SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));
                    pSSP->loopind = part_int + 1;

                    high_resolution_clock::time_point t1 = high_resolution_clock::now();

                    vector<double> new_mfpt(num_states);
                    vector<double> old_mfpt(num_states);


                    for (pair<double,int> it : pSSP->sweepedStates) {
                        old_mfpt[it.second] = it.first;
                    }

                    _prioritizedstates.clear();
                    optimalActionTransitionDistribution(_states);
                    pSSP->initTransMatrix();
                    pSSP->sweepedStates.clear();
                    double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);

                    high_resolution_clock::time_point t2 = high_resolution_clock::now();
                    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
                    cout << "\nTime Taken Iteration D MFPT-VI FPT Sub H1: " << duration/1000000.0 << " seconds" << endl;


                    for (pair<double,int> it : pSSP->sweepedStates) {
                        new_mfpt[it.second] = it.first;
                    }

                    vector<pair<double,int>> _new_sorted_states;
                    for(int t = 0; t < _states.size(); t++){
                        _new_sorted_states.push_back(make_pair(fabs(new_mfpt[t] - old_mfpt[t]), t));
                    }
                    std::sort(_new_sorted_states.begin(), _new_sorted_states.end(), [](pair<double,int> &left, pair<double,int> &right) {
                        return left.first < right.first;
                    });

                    for(pair<double,int> ele : _new_sorted_states){
                        cout << ele.second << " " << endl;
                    }
                    cout << endl;

                    //vector<int>stateid(num_states);
                    int it_cnt = 0;
                    for (pair<double,int> it : _new_sorted_states) {
                        stateid[it_cnt] = it.second;
                        it_cnt++;
                    }
                    _new_sorted_states.clear();

                    start_state = part_int*it_cnt/partition_num;
                    //start_state = 0;
                    end_state = (part_int == (partition_num - 1)) ? (((part_int + 1) * it_cnt)/partition_num) : ((((part_int+1)*it_cnt)/partition_num) + it_cnt/10);
                    //end_state = (((part_int + 1) * it_cnt)/partition_num);
                    cout << "Start: " << start_state << " End: " << end_state << endl;

                    high_resolution_clock::time_point t11 = high_resolution_clock::now();

                    for(int state_cnt = start_state; state_cnt < end_state; state_cnt++){
                        if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                            updateStateRecords(_states[stateid[state_cnt]]);
                        //else if((*itr)->type==START && (*itr)->spawn_parent)
                        //  updateStateRecords((*itr)->spawn_parent);
                        else
                            continue; //ignore goals or obstacles;
                    }//for

                    high_resolution_clock::time_point t22 = high_resolution_clock::now();
                    auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
                    cout << "\nTime Taken Iteration D MFPT-VI Sub H1 Bellman: " << duration1/1000000.0 << " seconds" << endl;
                }
                else{

                    //cout << it_cnt << endl;

                    for(int state_cnt = start_state; state_cnt < end_state; state_cnt++){
                        if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                            updateStateRecords(_states[stateid[state_cnt]]);
                        //else if((*itr)->type==START && (*itr)->spawn_parent)
                        //  updateStateRecords((*itr)->spawn_parent);
                        else
                            continue; //ignore goals or obstacles;
                    }//for
                }
            }
            else if(direction=="backward"){
                for(vector<mdp_state_t*>::reverse_iterator ritr=_states.rbegin(); ritr!=_states.rend(); ritr++){
                    if((*ritr)->type==BODY || (*ritr)->type==START)
                        updateStateRecords(*ritr);
                    //else if((*ritr)->type==START && (*ritr)->spawn_parent)
                    //  updateStateRecords((*ritr)->spawn_parent);
                    else
                        continue; //ignore goals or obstacles;
                }//for
            }
            else{
                cerr<<"Propagation direction!!"<<endl;
                assert(0);
            }

            //compare epsilon value and snapshot, for next iteration
            double max_delta=0;
            //for(uint i=0; i<_states.size(); i++){
            for(int state_cnt = start_state; state_cnt < end_state; state_cnt++){
                mdp_state_t* s=_states[stateid[state_cnt]];
                // for those spawn states
                if(s->spawn_parent){
                    s=s->spawn_parent;
                }
                double delta = fabs(s->optimal_value - s->last_optimal_value);
                max_delta = (delta>max_delta)? delta: max_delta;
                s->last_optimal_value = s->optimal_value;
                s->last_optimal_action = s->optimal_action;
            }

            cout << max_delta << endl;
            if(max_delta < 0.1)
                break;

        }

        cout << "Closed Partition: " << part_int+1 << " Loop: " << loop_cnt_part << endl;

    }

}


void
MDP::valueIterationMFPTDSubH2(vector<mdp_state_t*>& _states, string direction){

    vector<mdp_state_t*> _prioritizedstates;
    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    int partition_num = pParams->getParamNode()["environment"]["partitions"]["num_partitions"].as<int>();

    for(int part_int = 0; part_int < partition_num; part_int++){

        int loop_cnt_part = -1;

        int start_state, end_state=0;



        while (true) {

            loop_cnt_part++;

            if(direction=="forward"){

                if(loop_cnt_part >= 0 && loop_cnt_part < 10000 && loop_cnt_part % 2 == 0){// && loop_cnt_part <= 6){
                    SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));
                    pSSP->loopind = part_int + 1;

                    high_resolution_clock::time_point t1 = high_resolution_clock::now();

                    vector<double> new_mfpt(num_states);
                    vector<double> old_mfpt(num_states);


                    for (pair<double,int> it : pSSP->sweepedStates) {
                        old_mfpt[it.second] = it.first;
                    }

                    _prioritizedstates.clear();
                    optimalActionTransitionDistribution(_states);
                    pSSP->initTransMatrix();
                    pSSP->sweepedStates.clear();
                    double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);

                    high_resolution_clock::time_point t2 = high_resolution_clock::now();
                    auto duration = duration_cast<microseconds>( t2 - t1 ).count();
                    cout << "\nTime Taken Iteration D MFPT-VI FPT Sub H2: " << duration/1000000.0 << " seconds" << endl;


                    for (pair<double,int> it : pSSP->sweepedStates) {
                        new_mfpt[it.second] = it.first;
                    }

                    vector<pair<double,int>> _new_sorted_states;
                    for(int t = 0; t < _states.size(); t++){
                        _new_sorted_states.push_back(make_pair(fabs(new_mfpt[t] - old_mfpt[t]), t));
                    }
                    std::sort(_new_sorted_states.begin(), _new_sorted_states.end(), [](pair<double,int> &left, pair<double,int> &right) {
                        return left.first < right.first;
                    });

                    //                  for(pair<double,int> ele : _new_sorted_states){
                    //                      cout << ele.first << " ";
                    //                  }
                    //                  cout << endl;

                    //vector<int>stateid(num_states);
                    int it_cnt = 0;
                    double e_min = (_new_sorted_states[0]).first, e_max = (_new_sorted_states[_new_sorted_states.size() - 20]).first;
                    cout << "E Min: " << e_min << " , E Max: " << e_max << endl;
                    double intv = (e_max - e_min)/partition_num;
                    double start_value = e_min + (part_int*intv), end_value = e_min + ((part_int+1)*intv);

                    cout << "Start Value: " << start_value << " , End Value: " << end_value << endl;

                    for (pair<double,int> it : _new_sorted_states) {
                        stateid[it_cnt] = it.second;
                        it_cnt++;
                    }

                    start_state = 0, end_state = 0;
                    for (pair<double,int> it : _new_sorted_states) {
                        if(it.first > start_value)
                        {

                            break;
                        }
                        start_state++;
                    }
                    for (pair<double,int> it : _new_sorted_states) {
                        if(it.first > end_value)
                        {

                            break;
                        }
                        end_state++;
                    }
                    _new_sorted_states.clear();



                    end_state = (part_int == (partition_num - 1)) ? end_state : (end_state + it_cnt/250);
                    start_state--;
                    end_state--;

                    cout << "Start: " << start_state << " End: " << end_state << endl;

                    high_resolution_clock::time_point t11 = high_resolution_clock::now();

                    for(int state_cnt = start_state; state_cnt < end_state; state_cnt++){
                        if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                            updateStateRecords(_states[stateid[state_cnt]]);
                        //else if((*itr)->type==START && (*itr)->spawn_parent)
                        //  updateStateRecords((*itr)->spawn_parent);
                        else
                            continue; //ignore goals or obstacles;
                    }//for

                    high_resolution_clock::time_point t22 = high_resolution_clock::now();
                    auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
                    cout << "\nTime Taken Iteration D MFPT-VI Sub H2 Bellman: " << duration1/1000000.0 << " seconds" << endl;
                }
                else{

                    //cout << it_cnt << endl;

                    for(int state_cnt = start_state; state_cnt < end_state; state_cnt++){
                        if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START)
                            updateStateRecords(_states[stateid[state_cnt]]);
                        //else if((*itr)->type==START && (*itr)->spawn_parent)
                        //  updateStateRecords((*itr)->spawn_parent);
                        else
                            continue; //ignore goals or obstacles;
                    }//for
                }
            }
            else if(direction=="backward"){
                for(vector<mdp_state_t*>::reverse_iterator ritr=_states.rbegin(); ritr!=_states.rend(); ritr++){
                    if((*ritr)->type==BODY || (*ritr)->type==START)
                        updateStateRecords(*ritr);
                    //else if((*ritr)->type==START && (*ritr)->spawn_parent)
                    //  updateStateRecords((*ritr)->spawn_parent);
                    else
                        continue; //ignore goals or obstacles;
                }//for
            }
            else{
                cerr<<"Propagation direction!!"<<endl;
                assert(0);
            }

            //compare epsilon value and snapshot, for next iteration
            double max_delta=0;
            //for(uint i=0; i<_states.size(); i++){
            for(int state_cnt = start_state; state_cnt < end_state; state_cnt++){
                mdp_state_t* s=_states[stateid[state_cnt]];
                // for those spawn states
                if(s->spawn_parent){
                    s=s->spawn_parent;
                }
                double delta = fabs(s->optimal_value - s->last_optimal_value);
                max_delta = (delta>max_delta)? delta: max_delta;
                s->last_optimal_value = s->optimal_value;
                s->last_optimal_action = s->optimal_action;
            }

            cout << max_delta << endl;
            if(max_delta < 0.1)
                break;

        }

        cout << "Closed Partition: " << part_int+1 << " Loop: " << loop_cnt_part << endl;

    }

}


double
MDP::valueIterationLocal(vector<mdp_state_t*>& _states, string direction, int startstate, int finishstate){

    if(direction=="forward"){
        for(vector<mdp_state_t*>::iterator itr=_states.begin(); itr!=_states.end();itr++){
            if((*itr)->type==BODY || (*itr)->type==START){
                if((*itr)->id >= startstate && (*itr)->id <= finishstate){
                    updateStateRecords(*itr);
                }
            }

            //else if((*itr)->type==START && (*itr)->spawn_parent)
            //  updateStateRecords((*itr)->spawn_parent);
            else
                continue; //ignore goals or obstacles;
        }//for
    }
    else if(direction=="backward"){
        for(vector<mdp_state_t*>::reverse_iterator ritr=_states.rbegin(); ritr!=_states.rend(); ritr++){
            if((*ritr)->type==BODY || (*ritr)->type==START)
                updateStateRecords(*ritr);
            //else if((*ritr)->type==START && (*ritr)->spawn_parent)
            //  updateStateRecords((*ritr)->spawn_parent);
            else
                continue; //ignore goals or obstacles;
        }//for
    }
    else{
        cerr<<"Propagation direction!!"<<endl;
        assert(0);
    }

    //compare epsilon value and snapshot, for next iteration
    double max_delta=0;
    for(uint i=0; i<_states.size(); i++){
        mdp_state_t* s=_states[i];
        // for those spawn states
        if(s->spawn_parent){
            s=s->spawn_parent;
        }
        double delta = fabs(s->optimal_value - s->last_optimal_value);
        max_delta = (delta>max_delta)? delta: max_delta;
        s->last_optimal_value = s->optimal_value;
        s->last_optimal_action = s->optimal_action;
    }

    //cout<<"max_delta: "<<max_delta<<endl;
    return max_delta;

}


double
MDP::valueIterationLocalStates(vector<mdp_state_t*>& _states, vector<int> statestoupdate, string direction){

    if(direction=="forward"){
        for (vector<int>::iterator states_itr = statestoupdate.begin(); states_itr != statestoupdate.end(); states_itr++){
            for(vector<mdp_state_t*>::iterator itr=_states.begin(); itr!=_states.end();itr++){
                if((*itr)->type==BODY || (*itr)->type==START){
                    if((*itr)->id == (*states_itr)){
                        updateStateRecords(*itr);
                    }
                }

                //else if((*itr)->type==START && (*itr)->spawn_parent)
                //  updateStateRecords((*itr)->spawn_parent);
                else
                    continue; //ignore goals or obstacles;
            }//for
        }
    }
    else if(direction=="backward"){
        for(vector<mdp_state_t*>::reverse_iterator ritr=_states.rbegin(); ritr!=_states.rend(); ritr++){
            if((*ritr)->type==BODY || (*ritr)->type==START)
                updateStateRecords(*ritr);
            //else if((*ritr)->type==START && (*ritr)->spawn_parent)
            //  updateStateRecords((*ritr)->spawn_parent);
            else
                continue; //ignore goals or obstacles;
        }//for
    }
    else{
        cerr<<"Propagation direction!!"<<endl;
        assert(0);
    }

    //compare epsilon value and snapshot, for next iteration
    double max_delta=0;
    for(uint i=0; i<_states.size(); i++){
        mdp_state_t* s=_states[i];
        // for those spawn states
        if(s->spawn_parent){
            s=s->spawn_parent;
        }
        double delta = fabs(s->optimal_value - s->last_optimal_value);
        max_delta = (delta>max_delta)? delta: max_delta;
        s->last_optimal_value = s->optimal_value;
        s->last_optimal_action = s->optimal_action;
    }

    //cout<<"max_delta: "<<max_delta<<endl;
    return max_delta;

}

void
MDP::policyIteration(vector<mdp_state_t*>& _states)
{
    int size = _states.size();
    int t = 0;
    double value_total = 0;
    while (true)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();

        // policy evaluation
        for(mdp_state_t* state : _states)
        {
          if (state->type !=BODY && state->type!=START) // skip absorbing states
          {
              continue;
          }
          state->optimal_value = 0;
        }
        while (true)
        {
            value_total = 0;
            double delta = 0;
            for (mdp_state_t* state : _states)
            {
                if (state->type !=BODY && state->type!=START) // skip absorbing states
                {
                    continue;
                }
                double prev_value = state->optimal_value;
                if (state->successors[state->optimal_action] != state) // the act should be legal???
                {
                    Vec2 v_disturb = pDisturb->getVec(state->id);
                    getTransitionModel(state, state->optimal_action, v_disturb);
                }
                state->optimal_value = getQvalue(state, state->optimal_action); // in-place update
                delta = std::max(std::abs(prev_value - state->optimal_value), delta);
            }
            //      std::cout << "delta " << delta << std::endl;
            if (delta < epsilon)
            {
                //std::cout << "delta " << delta << std::endl;
                break;
            }
            for(mdp_state_t* state : _states)
            {
              if (state->type !=BODY && state->type!=START) // skip absorbing states
              {
                  continue;
              }
              value_total += state->optimal_value/size;
            }
            //cout<<"value at this iteration is"<<value_total<<endl;
        }
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        cout << "\nTime Taken Iteration PI Pol Eva: " << duration/1000000.0 << " seconds" << endl;

        high_resolution_clock::time_point t11 = high_resolution_clock::now();

        //std::cout << "policy evaluation done" << std::endl;

        bool policy_stable = true;
        // policy improvement
        int mismatch = 0;
        for (mdp_state_t* state : _states)
        {
            if(state->type !=BODY &&  state->type!=START) // skip absorbing states
            {
                continue;
            }
            Vec2 v_disturb = pDisturb->getVec(state->id);
            action_t prev_action = state->optimal_action;
            double max_value = std::numeric_limits<double>::lowest();
            for (unsigned int i = ZERO; i < NUM_ACTIONS; i++)
            {
                action_t action = static_cast<action_t> (i);
                if (state->successors[action] != state) // the act should be legal???
                {
                    getTransitionModel(state, action, v_disturb);
                    double value = getQvalue(state, action);

                    if (value >= max_value)
                    {
                        state->optimal_action = action;
                        max_value = value;
                        state->optimal_value = max_value;
                    }
                }
            }

            // set up only one optimal action
            std::fill(state->actions.begin(), state->actions.end(), false);
            state->actions[state->optimal_action] = true;
            //

            if (prev_action != state->optimal_action)
            {
                policy_stable = false;
                mismatch++;
            }
        }

        high_resolution_clock::time_point t22 = high_resolution_clock::now();
        auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
        cout << "\nTime Taken Iteration PI Pol Imp: " << duration1/1000000.0 << " seconds" << endl;

        //    for (int i = 3 - 1; i >= 0; i--)
        //    {
        //      for (int j = 0; j < 3; j++)
        //      {
        //        std::cout << _states[i * 3 + j]->optimal_action << " ";
        //      }
        //      std::cout << std::endl;
        //    }

        if (policy_stable)
        {
            std::cout << "policy iteration done in iteration: " << t << std::endl;
            return;
        }
        //std::cout << "policy improvement going on. Mismatch = " << mismatch << std::endl;
        std::cout << mismatch << std::endl;
        t++;

    }

    cerr << "This line is unreachable." << endl;
    return;
}


void
MDP::policyIterationPS(vector<mdp_state_t*>& _states)
{
    int t = 0;
    while (true)
    {
        vector<pair<double,mdp_state_t*>> _sorted_states;
        vector<pair<double,mdp_state_t*>> _new_sorted_states;
        for(vector<mdp_state_t*>::iterator itr=_states.begin(); itr!=_states.end();itr++){
            if((*itr)->type==BODY || (*itr)->type==START)
                _sorted_states.push_back(make_pair(0, *itr));
            else
                continue; //ignore goals or obstacles;
        }//for


        // policy evaluation
        while (true)
        {
            double delta = 0;

            for (pair<double,mdp_state_t*> state : _sorted_states)
            {
                if ((state.second)->type !=BODY && (state.second)->type!=START) // skip absorbing states
                {
                    continue;
                }
                double prev_value = (state.second)->optimal_value;
                if ((state.second)->successors[(state.second)->optimal_action] != (state.second)) // the act should be legal???
                {
                    Vec2 v_disturb = pDisturb->getVec((state.second)->id);
                    getTransitionModel((state.second), (state.second)->optimal_action, v_disturb);
                }
                (state.second)->optimal_value = getQvalue((state.second), (state.second)->optimal_action); // in-place update
                delta = std::max(std::abs(prev_value - (state.second)->optimal_value), delta);
                _new_sorted_states.push_back(make_pair(prev_value - (state.second)->optimal_value, state.second));
            }

            std::sort(_new_sorted_states.begin(), _new_sorted_states.end(), [](pair<double,mdp_state_t*> &left, pair<double,mdp_state_t*> &right) {
                return left.first > right.first;
            });

            _sorted_states = _new_sorted_states;
            _new_sorted_states.clear();

            //      std::cout << "delta " << delta << std::endl;
            if (delta < epsilon)
            {
                //std::cout << "delta " << delta << std::endl;
                break;
            }
        }
        std::cout << "policy evaluation done" << std::endl;

        bool policy_stable = true;
        // policy improvement
        int mismatch = 0;
        for (mdp_state_t* state : _states)
        {
            if(state->type !=BODY &&  state->type!=START) // skip absorbing states
            {
                continue;
            }
            Vec2 v_disturb = pDisturb->getVec(state->id);
            action_t prev_action = state->optimal_action;
            double max_value = std::numeric_limits<double>::lowest();
            for (unsigned int i = ZERO; i < NUM_ACTIONS; i++)
            {
                action_t action = static_cast<action_t> (i);
                if (state->successors[action] != state) // the act should be legal???
                {
                    getTransitionModel(state, action, v_disturb);
                    double value = getQvalue(state, action);

                    if (value >= max_value)
                    {
                        state->optimal_action = action;
                        max_value = value;
                        state->optimal_value = max_value;
                    }
                }
            }

            // set up only one optimal action
            std::fill(state->actions.begin(), state->actions.end(), false);
            state->actions[state->optimal_action] = true;
            //

            if (prev_action != state->optimal_action)
            {
                policy_stable = false;
                mismatch++;
            }
        }

        //    for (int i = 3 - 1; i >= 0; i--)
        //    {
        //      for (int j = 0; j < 3; j++)
        //      {
        //        std::cout << _states[i * 3 + j]->optimal_action << " ";
        //      }
        //      std::cout << std::endl;
        //    }

        if (policy_stable)
        {
            std::cout << "policy iteration done in iteration: " << t << std::endl;
            return;
        }
        std::cout << "policy improvement going on. Mismatch = " << mismatch << std::endl;
        std::cout << mismatch << std::endl;
        t++;

    }

    cerr << "This line is unreachable." << endl;
    return;
}


void
MDP::policyIterationMFPT(vector<mdp_state_t*>& _orderedstates)
{
    vector<mdp_state_t*> _prioritizedstates;
    vector<mdp_state_t*> _originalstates = _orderedstates;
    bool use_fpt = 1;
    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));


    SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));
    if(use_fpt == 1){

        high_resolution_clock::time_point t1 = high_resolution_clock::now();

        optimalActionTransitionDistribution(_orderedstates);
        pSSP->initTransMatrix();
        double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);
        pSSP->sweepedStates.clear();

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        cout << "\nTime Taken Iteration MFPT-VI FPT: " << duration/1000000.0 << " seconds" << endl;
    }

    vector<int> stateid(num_states);
    for (int it = 0; it < num_states; it++)
    {
        stateid[it] = it;
    }

    int t = 0;

    while (true)
    {
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        // policy evaluation
        while (true)
        {
            double delta = 0;
            //for (mdp_state_t* state : _orderedstates)
            for (int it = 0; it < num_states; it++)
            {
                mdp_state_t* state = pNet->getState(pNet->cell_centers[stateid[it]]);
                if (state->type !=BODY && state->type!=START) // skip absorbing states
                {
                    continue;
                }
                double prev_value = state->optimal_value;
                if (state->successors[state->optimal_action] != state) // the act should be legal???
                {
                    Vec2 v_disturb = pDisturb->getVec(state->id);
                    getTransitionModel(state, state->optimal_action, v_disturb);
                }
                state->optimal_value = getQvalue(state, state->optimal_action); // in-place update
                delta = std::max(std::abs(prev_value - state->optimal_value), delta);
            }
            //      std::cout << "delta " << delta << std::endl;
            if (delta < epsilon)
            {
                //std::cout << "delta " << delta << std::endl;
                break;
            }
        }

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        cout << "\nTime Taken Iteration MFPT-PI Pol Eva: " << duration/1000000.0 << " seconds" << endl;

        //std::cout << "policy evaluation done" << std::endl;

        high_resolution_clock::time_point t11 = high_resolution_clock::now();

        bool policy_stable = true;
        // policy improvement
        int mismatch = 0;
        for (int it = 0; it < num_states; it++)
        {
            mdp_state_t* state = pNet->getState(pNet->cell_centers[stateid[it]]);
            if(state->type !=BODY &&  state->type!=START) // skip absorbing states
            {
                continue;
            }
            Vec2 v_disturb = pDisturb->getVec(state->id);
            action_t prev_action = state->optimal_action;
            double max_value = std::numeric_limits<double>::lowest();
            for (unsigned int i = ZERO; i < NUM_ACTIONS; i++)
            {
                action_t action = static_cast<action_t> (i);
                if (state->successors[action] != state) // the act should be legal???
                {
                    getTransitionModel(state, action, v_disturb);
                    double value = getQvalue(state, action);

                    if (value >= max_value)
                    {
                        state->optimal_action = action;
                        max_value = value;
                        state->optimal_value = max_value;
                    }
                }
            }

            // set up only one optimal action
            std::fill(state->actions.begin(), state->actions.end(), false);
            state->actions[state->optimal_action] = true;
            //

            if (prev_action != state->optimal_action)
            {
                policy_stable = false;
                mismatch++;
            }
        }

        high_resolution_clock::time_point t22 = high_resolution_clock::now();
        auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
        cout << "\nTime Taken Iteration MFPT-PI Pol Imp: " << duration1/1000000.0 << " seconds" << endl;

        if (policy_stable)
        {
            std::cout << "policy iteration done with iteration: " << t << std::endl;
            return;
        }
        //std::cout << "policy improvement going on. Mismatch = " << mismatch << std::endl;
        std::cout << mismatch << std::endl;

        if(t<=10 && use_fpt == 1){

            high_resolution_clock::time_point t1 = high_resolution_clock::now();

            _prioritizedstates.clear();
            optimalActionTransitionDistribution(_orderedstates);
            pSSP->initTransMatrix();
            double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);

            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>( t2 - t1 ).count();
            cout << "\nTime Taken Iteration MFPT-PI FPT: " << duration/1000000.0 << " seconds" << endl;

            //        _orderedstates.clear();
            //        //cout << "SSP Calculated Outside T: " << t << endl;
            //        int it_cnt = 0;
            //        for (pair<double,int> it : pSSP->sweepedStates) {
            //            stateid[it_cnt] = it.second;
            //            //cout << stateid[it_cnt] << endl;
            //            it_cnt++;
            //            //mdp_state_t* new_state = pNet->getState(pNet->cell_centers[it.second]);
            //        }
            //        pSSP->sweepedStates.clear();
            //        //cout << "Sweeping Done Outside T: " << t << endl;
        }
        else{
            _orderedstates = _originalstates;
        }


        t++;
    }

    cerr << "This line is unreachable." << endl;
    return;
}


void
MDP::policyIterationLP(vector<mdp_state_t*>& _states)
{
    int t = 0;

    while (true)
    {
        int k = 0;

        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        while(true){

            // policy evaluation
            double gamma = 1;
            double delta = 0;

            Eigen::MatrixXd A(_states.size(), _states.size());

            for(int i = 0; i < _states.size(); i++){
                for(int j = 0; j < _states.size(); j++){
                    A(i,j) = 0;
                }
            }

            for(int i = 0; i < _states.size(); i++){
                if (_states[i]->type !=BODY && _states[i]->type!=START) // skip absorbing states
                {
                    continue;
                }
                A(i,i) = 1;
                Vec2 v_disturb = pDisturb->getVec(_states[i]->id);
                for(uint j=NORTH; j<NUM_ACTIONS; j++){
                    action_t act = (action_t)j;
                    if(act == _states[i]->optimal_action){
                        if(_states[i]->successors[act] != _states[i]) //the act should be legal
                        {
                            getTransitionModel(_states[i], act, v_disturb);
                            A(i,_states[i]->successors[act]->id) += -1 * gamma * _states[i]->probs[act];
                        }
                    }
                }
            }

            //        for(int i = 0; i < _states.size(); i++){
            //            for(int j = 0; j < _states.size(); j++){
            //                cout << A(i,j) << " ";
            //            }
            //            cout << endl;
            //        }

            Eigen::VectorXd b(_states.size());

            for(int i = 0; i < _states.size(); i++){
                if (_states[i]->type !=BODY && _states[i]->type!=START) // skip absorbing states
                {
                    continue;
                }
                mdp_state_t* s = _states[i];
                Vec2 v_disturb = pDisturb->getVec(s->id);
                double b_val = 0;

                for(uint j=NORTH; j<NUM_ACTIONS; j++){
                    action_t act = (action_t)j;
                    if(act == s->optimal_action){
                        if(s->successors[act] != s) //the act should be legal
                        {
                            getTransitionModel(s, act, v_disturb);
                            double qval = getQvalue(s, act);
                            if(s->successors[act]->type == GOAL){
                                b_val += (s->probs[act] * 100);
                                //cout << s->id << " : " << b_val << endl;
                            }
                            else if(s->successors[act]->type == OBSTACLE){
                                b_val += (s->probs[act] * -100);
                                //cout << s->id << " : " << b_val << endl;
                            }
                            else
                                b_val += (s->probs[act] * 0);

                        }
                    }
                }
                b[i] = b_val;

            }

            //        for(int i = 0; i < _states.size(); i++){
            //            cout << b[i];
            //            cout << endl;
            //        }

            Eigen::VectorXd v = A.fullPivLu().solve(b);

            for(int i = 0; i < _states.size(); i++){
                if (_states[i]->type !=BODY && _states[i]->type!=START) // skip absorbing states
                {
                    continue;
                }
                double prev_value = _states[i]->optimal_value;
                //double prev_value = getQvalue(_states[i], _states[i]->optimal_action);
                _states[i]->optimal_value = v[i];
                delta = std::max(std::abs(prev_value - _states[i]->optimal_value), delta);
            }
            std::cout << "k: " << k << " , delta " << delta << std::endl;

            if (delta < epsilon)
            {
                //std::cout << "delta " << delta << std::endl;
                break;
            }
            k++;
        }

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>( t2 - t1 ).count();
        cout << "\nTime Taken Iteration PI-Linear Pol Eva: " << duration/1000000.0 << " seconds" << endl;


        std::cout << "policy evaluation done: " << t << std::endl;

        high_resolution_clock::time_point t11 = high_resolution_clock::now();

        bool policy_stable = true;
        // policy improvement
        int mismatch = 0;
        for (mdp_state_t* state : _states)
        {
            if(state->type !=BODY &&  state->type!=START) // skip absorbing states
            {
                continue;
            }
            Vec2 v_disturb = pDisturb->getVec(state->id);
            action_t prev_action = state->optimal_action;
            double max_value = std::numeric_limits<double>::lowest();
            for (unsigned int i = ZERO; i < NUM_ACTIONS; i++)
            {
                action_t action = static_cast<action_t> (i);
                if (state->successors[action] != state) // the act should be legal???
                {
                    getTransitionModel(state, action, v_disturb);
                    double value = getQvalue(state, action);

                    if (value >= max_value)
                    {
                        state->optimal_action = action;
                        max_value = value;
                        state->optimal_value = max_value;
                    }
                }
            }

            // set up only one optimal action
            std::fill(state->actions.begin(), state->actions.end(), false);
            state->actions[state->optimal_action] = true;
            //

            if (prev_action != state->optimal_action)
            {
                policy_stable = false;
                mismatch++;
            }
        }

        high_resolution_clock::time_point t22 = high_resolution_clock::now();
        auto duration1 = duration_cast<microseconds>( t22 - t11 ).count();
        cout << "\nTime Taken Iteration PI-Linear Pol Imp: " << duration1/1000000.0 << " seconds" << endl;


        //    for (int i = 3 - 1; i >= 0; i--)
        //    {
        //      for (int j = 0; j < 3; j++)
        //      {
        //        std::cout << _states[i * 3 + j]->optimal_action << " ";
        //      }
        //      std::cout << std::endl;
        //    }

        //policy_stable = false;
        if (policy_stable && t > 3 )
            //if (mismatch <= 6 && t > 3 )
        {
            std::cout << "policy iteration done in iteration: " << t << ". Mismatch = " << mismatch << std::endl;
            return;
        }
        std::cout << "policy improvement going on. " << " t: " << t << " Mismatch = " << mismatch << std::endl;
        std::cout << mismatch << std::endl;
        t++;

    }

    cerr << "This line is unreachable." << endl;
    return;
}


bool
MDP::pairCompare(const std::pair<int, mdp_state_t*>& firstElem, const std::pair<int, mdp_state_t*>& secondElem) {
    return (firstElem.second)->q_values[firstElem.first] > (secondElem.second)->q_values[secondElem.first];
}


double
MDP::QLearning(vector<mdp_state_t*>& _states){

    vector<int>stateid(num_states);
    int ROWSIZE = pParams->getParamNode()["environment"]["grids"]["num_rows"].as<int>();
    int COLUMNSIZE = pParams->getParamNode()["environment"]["grids"]["num_cols"].as<int>();
    int NUM_ACTIONS = 9;
    int num_iterations = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
    int trials = pParams->getParamNode()["mdp_methods"]["num_trials"].as<unsigned int>();
    double learning_rate = pParams->getParamNode()["mdp_methods"]["learning_rate"].as<double>();
    double discount_factor = pParams->getParamNode()["mdp_methods"]["discount_factor"].as<double>();
    double action_cost = -1.0 * pParams->getParamNode()["mdp_methods"]["action_cost"].as<double>();
    double goal_reward = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
    double explore_rate = pParams->getParamNode()["mdp_methods"]["explore_rate"].as<double>();

    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

    // Define start and goal
    int start = -1;
    int goal = pNet->getState(tf2_goals_1[0].translation)->id;

    // Setting visited flag to 0
    double visited[ROWSIZE*COLUMNSIZE];
    for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
    {
        visited[i] = 0;
    }

    double tot_val_diff = 0;
    vector<double> tot_val_diff_vec;

    for (int t = 1; t <= trials; t++)
    {
        std::cout << "\n\nTrial " << t << "\n\n";
        int current = start;
        while (true)
        {
            current = rand() % (ROWSIZE*COLUMNSIZE);
            if (current != goal && _states[current]->type != OBSTACLE) break;
        }

        tot_val_diff = 0;

        for (int k = 1; k <= num_iterations; k++)
        {

            mdp_state_t* st = _states[current];
            visited[current] = 1;
            vector<pair<int, mdp_state_t*> > nextStateValue;

            for(int j = NORTH; j < NUM_ACTIONS; j++){
                if(((st->successors[j])->id != st->id) && (visited[(st->successors[j])->id] != 1))
                    nextStateValue.push_back(make_pair(j,st));
            }

            if (nextStateValue.empty()) break;
            std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

            // Exploration-Exploitation TradeOff
            pair<int, mdp_state_t*> nextState = nextStateValue[0];
            int action;
            int p = min((rand() % nextStateValue.size()), nextStateValue.size() - 1);
            action = (rand() % 100 < explore_rate) ? nextStateValue[p].first : nextState.first;

            // Next Action
            while((st->successors[action])->id == st->id || (st->successors[action])->type == OBSTACLE){

                p = min((rand() % NUM_ACTIONS), NUM_ACTIONS - 1);
                action = (rand() % 100 < explore_rate) ? p : nextState.first;

            }
            mdp_state_t* next_st = (st->successors[action]);

            // Next State Max Value
            double value = -INF;
            for (int i = 1; i < NUM_ACTIONS; i++)
            {
                if (next_st->q_values[i] > value)
                    value = next_st->q_values[i];
            }

            if (t <= 5 * trials / 5) {

                // Calculate immediate reward
                double rew;
                if((st->successors[action])->type == OBSTACLE) rew = action_cost;
                else if((st->successors[action])->type == GOAL) rew = goal_reward;
                else rew = action_cost;


                // Q-Update
                double newq_valuescalc = st->q_values[action] + learning_rate * (rew + (discount_factor * value) - st->q_values[action]);
                tot_val_diff += fabs(newq_valuescalc - st->q_values[action]);
                st->q_values[action] = newq_valuescalc;

                // Updating transition probability
                st->action_count[action]++;
                int tot_action_cnt = 0;
                for(int j = NORTH; j < NUM_ACTIONS; j++){
                    tot_action_cnt += st->action_count[j];
                }
                st->probs[action] = (st->action_count[action] * 1.0)/tot_action_cnt;
                st->post_probs[action] = (st->action_count[action] * 1.0)/tot_action_cnt;
            }

            if (next_st->id == goal) break;
            current = (st->successors[action])->id;


        }

        tot_val_diff_vec.push_back(tot_val_diff);

        // Reset visited flag
        for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
        {
            visited[i] = 0;
        }

        cout << tot_val_diff << endl;
        tot_val_diff_vec.push_back(tot_val_diff);

        vector<double> y(tot_val_diff_vec.end() - 70, tot_val_diff_vec.end());
        double sum_of_elems = 0;
        for (auto& n : y)
            sum_of_elems += n;

        if (sum_of_elems/70.0 == 0)
            break;

    }


    // Calculating Optimal Policy at each state
    vector<mdp_state_t*>& _presentStates = pNet->mdp_states;
    for(vector<mdp_state_t*>::iterator itr=_presentStates.begin(); itr!=_presentStates.end();itr++){

        if((*itr)->type == OBSTACLE || (*itr)->type == GOAL)
            continue;

        vector<pair<int, mdp_state_t*> > nextStateValue;

        for(int j = NORTH; j < NUM_ACTIONS; j++){
            if((((*itr)->successors[j])->id != (*itr)->id) && (visited[((*itr)->successors[j])->id] != 1))
                nextStateValue.push_back(make_pair(j,(*itr)));
        }

        if (nextStateValue.empty()) break;
        std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

        pair<int, mdp_state_t*> nextState = nextStateValue[0];
        (*itr)->optimal_action = (action_t)(nextState.first);
        (*itr)->optimal_value = (nextState.second)->q_values[nextState.first];
        std::fill((*itr)->actions.begin(), (*itr)->actions.end(), false);
        (*itr)->actions[nextState.first] = true;

    }

    // Writing Val Diff values to a file
    ofstream myfile;
    myfile.open ("val_diff_q.txt");

    cout << "Val Diff:" << endl;
    for(int rew_id = 0; rew_id < tot_val_diff_vec.size(); rew_id++){
        myfile << tot_val_diff_vec[rew_id];
        if(rew_id != tot_val_diff_vec.size()-1)
            myfile << ", ";
    }
    myfile << endl;
    myfile.close();

    return 0;

}


double
MDP::MFPT_QL(vector<mdp_state_t*>& _states){

    vector<int>stateid(num_states);
    int ROWSIZE = pParams->getParamNode()["environment"]["grids"]["num_rows"].as<int>();
    int COLUMNSIZE = pParams->getParamNode()["environment"]["grids"]["num_cols"].as<int>();
    int NUM_ACTIONS = 9;
    int num_iterations = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
    int trials = pParams->getParamNode()["mdp_methods"]["num_trials"].as<unsigned int>();
    double learning_rate = pParams->getParamNode()["mdp_methods"]["learning_rate"].as<double>();
    double discount_factor = pParams->getParamNode()["mdp_methods"]["discount_factor"].as<double>();
    double action_cost = -1.0 * pParams->getParamNode()["mdp_methods"]["action_cost"].as<double>();
    double goal_reward = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
    double explore_rate = pParams->getParamNode()["mdp_methods"]["explore_rate"].as<double>();
    int mfpt_frequency = pParams->getParamNode()["mdp_methods"]["mfpt_frequency"].as<unsigned int>();


    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

    // Setting Start and Goal
    int start = -1;
    int goal = pNet->getState(tf2_goals_1[0].translation)->id;

    // Set visited to 0
    double visited[ROWSIZE*COLUMNSIZE];
    for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
    {
        visited[i] = 0;
    }

    double tot_val_diff = 0;
    int flag = 0;
    vector<double> tot_val_diff_vec;

    for (int t = 1; t <= trials; t++)
    {
        std::cout << "\n\nTrial " << t << "\n\n";
        int current = start;
        while (true)
        {
            current = rand() % (ROWSIZE*COLUMNSIZE);
            if (current != goal && _states[current]->type != OBSTACLE) break;
        }

        tot_val_diff = 0;

        for (int k = 1; k <= num_iterations; k++)
        {


            mdp_state_t* st = _states[current];
            visited[current] = 1;
            vector<pair<int, mdp_state_t*> > nextStateValue;


            for(int j = NORTH; j < NUM_ACTIONS; j++){
                //cout << (st->successors[j])->id << ", ";
                if(((st->successors[j])->id != st->id) && (visited[(st->successors[j])->id] != 1))
                    nextStateValue.push_back(make_pair(j,st));
            }


            if (nextStateValue.empty()) break;
            std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

            // Exploration-Exploitation TradeOff
            pair<int, mdp_state_t*> nextState = nextStateValue[0];
            int action;
            int p = min((rand() % nextStateValue.size()), nextStateValue.size() - 1);
            action = (rand() % 100 < explore_rate) ? nextStateValue[p].first : nextState.first;

            // Next action
            while((st->successors[action])->id == st->id || (st->successors[action])->type == OBSTACLE){

                p = min((rand() % NUM_ACTIONS), NUM_ACTIONS - 1);
                action = (rand() % 100 < explore_rate) ? p : nextState.first;

            }

            // Next State Max Value
            mdp_state_t* next_st = (st->successors[action]);
            double value = -INF;
            for (int i = 1; i < NUM_ACTIONS; i++)
            {
                if (next_st->q_values[i] > value)
                    value = next_st->q_values[i];
            }

            if (t <= 5 * trials / 5) {

                // Calculate immediate reward
                double rew;
                if((st->successors[action])->type == OBSTACLE) rew = action_cost;
                else if((st->successors[action])->type == GOAL) rew = goal_reward;
                else rew = action_cost;

                // Q-Update
                double newq_valuescalc = st->q_values[action] + learning_rate * (rew + (discount_factor * value) - st->q_values[action]);
                tot_val_diff += fabs(newq_valuescalc - st->q_values[action]);
                st->q_values[action] = newq_valuescalc;

                // Updating transition probability
                st->action_count[action]++;
                int tot_action_cnt = 0;
                for(int j = NORTH; j < NUM_ACTIONS; j++){
                    tot_action_cnt += st->action_count[j];
                }

                for(int j = NORTH; j < NUM_ACTIONS; j++){
                    st->probs[j] = (st->action_count[j] * 1.0)/tot_action_cnt;
                    st->post_probs[j] = (st->action_count[j] * 1.0)/tot_action_cnt;
                }
            }

            if (next_st->id == goal) break;
            current = (st->successors[action])->id;


        }

        // Reset visited to 0
        for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
        {
            visited[i] = 0;
        }


        /* MFPT RELATED CODE */
        if(t % mfpt_frequency == 0 ) { //&& t == trials){
            cout << "Running MFPT...." << endl;
            pSSP->sweepedStates.clear();

            pSSP->initTransMatrix();
            double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);

            it_cnt = 0;
            for (pair<double,int> it : pSSP->sweepedStates) {
                stateid[it_cnt] = it.second;
                it_cnt++;
            }

            // MFPT-VI
            for(int itrr = 1; itrr <= 2; itrr++){
                for(int state_cnt = 0; state_cnt < it_cnt; state_cnt++){
                    if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START){
                        double old_val = _states[stateid[state_cnt]]->optimal_value;
                        updateStateRecordsQLearning(_states[stateid[state_cnt]]);
                        tot_val_diff += fabs(_states[stateid[state_cnt]]->optimal_value - old_val);

                    }
                    else
                        continue; //ignore goals or obstacles;
                }//for

                if(tot_val_diff == 0){
                    flag = 1;
                    break;
                }

                tot_val_diff = 0;
            }

        }

        cout << tot_val_diff << endl;
        tot_val_diff_vec.push_back(tot_val_diff);
        if(flag == 1)
            break;

    }


    // Calculating Optimal Policy at each state
    vector<mdp_state_t*>& _presentStates = pNet->mdp_states;
    for(vector<mdp_state_t*>::iterator itr=_presentStates.begin(); itr!=_presentStates.end();itr++){

        if((*itr)->type == OBSTACLE || (*itr)->type == GOAL)
            continue;

        vector<pair<int, mdp_state_t*> > nextStateValue;

        for(int j = NORTH; j < NUM_ACTIONS; j++){
            if((((*itr)->successors[j])->id != (*itr)->id) && (visited[((*itr)->successors[j])->id] != 1))
                nextStateValue.push_back(make_pair(j,(*itr)));
        }

        if (nextStateValue.empty()) break;
        std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

        pair<int, mdp_state_t*> nextState = nextStateValue[0];
        (*itr)->optimal_action = (action_t)(nextState.first);
        (*itr)->optimal_value = (nextState.second)->q_values[nextState.first];
        std::fill((*itr)->actions.begin(), (*itr)->actions.end(), false);
        (*itr)->actions[nextState.first] = true;

    }


    // Dislaying order of states in MFPT
    cout << endl;
    for(int state_cnt = 0; state_cnt < it_cnt; state_cnt++){
        cout << stateid[state_cnt] << ',';
        if(state_cnt %30 == 0)
            cout << endl;
    }

    // Writing Val Diff values to a file
    ofstream myfile;
    myfile.open ("val_diff_mfptq.txt");

    cout << "Val Diff:" << endl;
    for(int rew_id = 0; rew_id < tot_val_diff_vec.size(); rew_id++){
        myfile << tot_val_diff_vec[rew_id];
        if(rew_id != tot_val_diff_vec.size()-1)
            myfile << ", ";
    }
    myfile << endl;
    myfile.close();

    return 0;

}


double
MDP::DYNA(vector<mdp_state_t*>& _states){

    int ROWSIZE = pParams->getParamNode()["environment"]["grids"]["num_rows"].as<int>();
    int COLUMNSIZE = pParams->getParamNode()["environment"]["grids"]["num_cols"].as<int>();
    int NUM_ACTIONS = 9;
    int num_iterations = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
    int trials = pParams->getParamNode()["mdp_methods"]["num_trials"].as<unsigned int>();
    double learning_rate = pParams->getParamNode()["mdp_methods"]["learning_rate"].as<double>();
    double discount_factor = pParams->getParamNode()["mdp_methods"]["discount_factor"].as<double>();
    double action_cost = -1.0 * pParams->getParamNode()["mdp_methods"]["action_cost"].as<double>();
    double goal_reward = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
    double explore_rate = pParams->getParamNode()["mdp_methods"]["explore_rate"].as<double>();


    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

    // Set start and goal
    int start = -1;
    int goal = pNet->getState(tf2_goals_1[0].translation)->id;

    // Setting visited to 0
    double visited[ROWSIZE*COLUMNSIZE];
    for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
    {
        visited[i] = 0;
    }


    double tot_val_diff = 0;
    vector<double> tot_val_diff_vec;

    for (int t = 1; t <= trials; t++)
    {
        std::cout << "\n\nTrial " << t << "\n\n";
        int current = start;
        while (true)
        {
            current = rand() % (ROWSIZE*COLUMNSIZE);
            if (current != goal && _states[current]->type != OBSTACLE) break;
        }

        tot_val_diff = 0;

        for (int k = 1; k <= num_iterations; k++)
        {
            mdp_state_t* st = _states[current];
            visited[current] = 1;
            vector<pair<int, mdp_state_t*> > nextStateValue;


            for(int j = NORTH; j < NUM_ACTIONS; j++){
                if(((st->successors[j])->id != st->id) && (visited[(st->successors[j])->id] != 1))
                    nextStateValue.push_back(make_pair(j,st));
            }

            if (nextStateValue.empty()) break;
            std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

            // Exploration-Exploitation TradeOff
            pair<int, mdp_state_t*> nextState = nextStateValue[0];
            int action;
            int p = min((rand() % nextStateValue.size()), nextStateValue.size() - 1);
            action = (rand() % 100 < explore_rate) ? nextStateValue[p].first : nextState.first;

            // Next action
            while((st->successors[action])->id == st->id || (st->successors[action])->type == OBSTACLE){

                p = min((rand() % NUM_ACTIONS), NUM_ACTIONS - 1);
                action = (rand() % 100 < explore_rate) ? p : nextState.first;

            }

            // Next State Max Value
            mdp_state_t* next_st = (st->successors[action]);
            double value = -INF;
            for (int i = 1; i < NUM_ACTIONS; i++)
            {
                if (next_st->q_values[i] > value)
                    value = next_st->q_values[i];
            }

            // Calculate immediate reward
            double rew;
            if((st->successors[action])->type == OBSTACLE) rew = action_cost;
            else if((st->successors[action])->type == GOAL) rew = goal_reward;
            else rew = action_cost;

            // Q-Update
            double newq_valuescalc = st->q_values[action] + learning_rate * (rew + (discount_factor * value) - st->q_values[action]);
            tot_val_diff += fabs(newq_valuescalc - st->q_values[action]);
            st->q_values[action] = newq_valuescalc;

            // Updating transition probability
            st->action_count[action]++;
            int tot_action_cnt = 0;
            for(int j = NORTH; j < NUM_ACTIONS; j++){
                tot_action_cnt += st->action_count[j];
            }
            for(int j = NORTH; j < NUM_ACTIONS; j++){
                st->probs[j] = (st->action_count[j] * 1.0)/tot_action_cnt;
                st->post_probs[j] = (st->action_count[j] * 1.0)/tot_action_cnt;
            }

            if (next_st->id == goal) break;
            current = (st->successors[action])->id;

            // Random Exploration
            for(int rand_exp = 1; rand_exp <= 10; rand_exp++){

                int rand_current_state;
                while (true)
                {
                    rand_current_state = rand() % (ROWSIZE*COLUMNSIZE);

                    if (rand_current_state != goal && _states[rand_current_state]->type != OBSTACLE) break;
                }

                int rand_current_action;
                while (true)
                {
                    rand_current_action = 1 + (rand() % (NUM_ACTIONS-1));

                    if (((_states[rand_current_state]->successors[rand_current_action])->id != _states[rand_current_state]->id) &&
                            _states[rand_current_state]->successors[rand_current_action]->type != GOAL &&
                            _states[rand_current_state]->successors[rand_current_action]->type != OBSTACLE) break;
                }

                mdp_state_t* random_next_st = (_states[rand_current_state]->successors[rand_current_action]);
                double value = -INF;
                for (int i = 1; i < NUM_ACTIONS; i++)
                {
                    if (random_next_st->q_values[i] > value)
                        value = random_next_st->q_values[i];
                }

                double rew;
                if((_states[rand_current_state]->successors[rand_current_action])->type == OBSTACLE) rew = action_cost;
                else if((_states[rand_current_state]->successors[rand_current_action])->type == GOAL) rew = goal_reward;
                else rew = action_cost;

                double newq_valuescalc = _states[rand_current_state]->q_values[rand_current_action] + learning_rate * (rew + (discount_factor * value) - _states[rand_current_state]->q_values[rand_current_action]);
                tot_val_diff += fabs(newq_valuescalc - _states[rand_current_state]->q_values[rand_current_action]);
                _states[rand_current_state]->q_values[rand_current_action] = newq_valuescalc;

            }


        }

        // Reset visited to 0
        for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
        {
            visited[i] = 0;
        }

        cout << tot_val_diff << endl;
        tot_val_diff_vec.push_back(tot_val_diff);

        vector<double> y(tot_val_diff_vec.end() - 70, tot_val_diff_vec.end());
        double sum_of_elems = 0;
        for (auto& n : y)
            sum_of_elems += n;

        if (sum_of_elems/70.0 == 0)
            break;
    }

    // Calculating Optimal Policy at each state
    vector<mdp_state_t*>& _presentStates = pNet->mdp_states;
    for(vector<mdp_state_t*>::iterator itr=_presentStates.begin(); itr!=_presentStates.end();itr++){

        if((*itr)->type == OBSTACLE || (*itr)->type == GOAL)
            continue;

        vector<pair<int, mdp_state_t*> > nextStateValue;

        for(int j = NORTH; j < NUM_ACTIONS; j++){
            if((((*itr)->successors[j])->id != (*itr)->id) && (visited[((*itr)->successors[j])->id] != 1))
                nextStateValue.push_back(make_pair(j,(*itr)));
        }

        if (nextStateValue.empty()) break;
        std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

        pair<int, mdp_state_t*> nextState = nextStateValue[0];
        (*itr)->optimal_action = (action_t)(nextState.first);
        (*itr)->optimal_value = (nextState.second)->q_values[nextState.first];
        std::fill((*itr)->actions.begin(), (*itr)->actions.end(), false);
        (*itr)->actions[nextState.first] = true;

    }

    // Writing Val Diff values to a file
    ofstream myfile;
    myfile.open ("val_diff_dyna.txt");

    cout << "Val Diff:" << endl;
    for(int rew_id = 0; rew_id < tot_val_diff_vec.size(); rew_id++){
        myfile << tot_val_diff_vec[rew_id];
        if(rew_id != tot_val_diff_vec.size()-1)
            myfile << ", ";
    }
    myfile << endl;
    myfile.close();

    return 0;

}



double
MDP::MFPT_DYNA(vector<mdp_state_t*>& _states){

    vector<int>stateid(num_states);
    int ROWSIZE = pParams->getParamNode()["environment"]["grids"]["num_rows"].as<int>();
    int COLUMNSIZE = pParams->getParamNode()["environment"]["grids"]["num_cols"].as<int>();
    int NUM_ACTIONS = 9;
    int num_iterations = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
    int trials = pParams->getParamNode()["mdp_methods"]["num_trials"].as<unsigned int>();
    double learning_rate = pParams->getParamNode()["mdp_methods"]["learning_rate"].as<double>();
    double discount_factor = pParams->getParamNode()["mdp_methods"]["discount_factor"].as<double>();
    double action_cost = -1.0 * pParams->getParamNode()["mdp_methods"]["action_cost"].as<double>();
    double goal_reward = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
    double explore_rate = pParams->getParamNode()["mdp_methods"]["explore_rate"].as<double>();
    int mfpt_frequency = pParams->getParamNode()["mdp_methods"]["mfpt_frequency"].as<unsigned int>();

    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

    // Set start and goal
    int start = -1;
    int goal = pNet->getState(tf2_goals_1[0].translation)->id;

    // Setting visited flag to 0
    double visited[ROWSIZE*COLUMNSIZE];
    for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
    {
        visited[i] = 0;
    }

    double tot_val_diff = 0;
    int flag = 0;
    vector<double> tot_val_diff_vec;

    for (int t = 1; t <= trials; t++)
    {
        std::cout << "\n\nTrial " << t << "\n\n";
        int current = start;
        while (true)
        {
            current = rand() % (ROWSIZE*COLUMNSIZE);
            if (current != goal && _states[current]->type != OBSTACLE) break;
        }


        for (int k = 1; k <= num_iterations; k++)
        {


            mdp_state_t* st = _states[current];
            visited[current] = 1;
            vector<pair<int, mdp_state_t*> > nextStateValue;

            for(int j = NORTH; j < NUM_ACTIONS; j++){
                if(((st->successors[j])->id != st->id) && (visited[(st->successors[j])->id] != 1))
                    nextStateValue.push_back(make_pair(j,st));
            }


            if (nextStateValue.empty()) break;
            std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

            // Exploration-Exploitation TradeOff
            pair<int, mdp_state_t*> nextState = nextStateValue[0];
            int action;
            int p = min((rand() % nextStateValue.size()), nextStateValue.size() - 1);
            action = (rand() % 100 < explore_rate) ? nextStateValue[p].first : nextState.first;

            // Next action
            while((st->successors[action])->id == st->id || (st->successors[action])->type == OBSTACLE){

                p = min((rand() % NUM_ACTIONS), NUM_ACTIONS - 1);
                action = (rand() % 100 < explore_rate) ? p : nextState.first;

            }

            // Next Max-Val
            mdp_state_t* next_st = (st->successors[action]);
            double value = -INF;
            for (int i = 1; i < NUM_ACTIONS; i++)
            {
                if (next_st->q_values[i] > value)
                    value = next_st->q_values[i];
            }

            // Calculating immediate reward
            double rew;
            if((st->successors[action])->type == OBSTACLE) rew = action_cost;
            else if((st->successors[action])->type == GOAL) rew = goal_reward;
            else rew = action_cost;

            // Q-Update
            double newq_valuescalc = st->q_values[action] + learning_rate * (rew + (discount_factor * value) - st->q_values[action]);
            tot_val_diff += fabs(newq_valuescalc - st->q_values[action]);
            st->q_values[action] = newq_valuescalc;

            // Updating transition probability
            st->action_count[action]++;
            int tot_action_cnt = 0;
            for(int j = NORTH; j < NUM_ACTIONS; j++){
                tot_action_cnt += st->action_count[j];
            }
            st->probs[action] = (st->action_count[action] * 1.0)/tot_action_cnt;
            st->post_probs[action] = (st->action_count[action] * 1.0)/tot_action_cnt;


            if (next_st->id == goal) break;
            current = (st->successors[action])->id;

            // Random Exploration
            for(int rand_exp = 1; rand_exp <= 10; rand_exp++){

                int rand_current_state;
                while (true)
                {
                    rand_current_state = rand() % (ROWSIZE*COLUMNSIZE);

                    if (rand_current_state != goal && _states[rand_current_state]->type != OBSTACLE) break;
                }

                int rand_current_action;
                while (true)
                {
                    rand_current_action = 1 + (rand() % (NUM_ACTIONS-1));

                    if (((_states[rand_current_state]->successors[rand_current_action])->id != _states[rand_current_state]->id) &&
                            _states[rand_current_state]->successors[rand_current_action]->type != GOAL &&
                            _states[rand_current_state]->successors[rand_current_action]->type != OBSTACLE) break;
                }

                mdp_state_t* random_next_st = (_states[rand_current_state]->successors[rand_current_action]);
                double value = -INF;
                for (int i = 1; i < NUM_ACTIONS; i++)
                {
                    if (random_next_st->q_values[i] > value)
                        value = random_next_st->q_values[i];
                }

                double rew;
                if((_states[rand_current_state]->successors[rand_current_action])->type == OBSTACLE) rew = action_cost;
                else if((_states[rand_current_state]->successors[rand_current_action])->type == GOAL) rew = goal_reward;
                else rew = action_cost;



                double newq_valuescalc = _states[rand_current_state]->q_values[rand_current_action] + learning_rate * (rew + (discount_factor * value) - _states[rand_current_state]->q_values[rand_current_action]);
                tot_val_diff += fabs(newq_valuescalc - _states[rand_current_state]->q_values[rand_current_action]);
                _states[rand_current_state]->q_values[rand_current_action] = newq_valuescalc;

            }


        }

        // Reset visited to 0
        for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
        {
            visited[i] = 0;
        }


        /* MFPT RELATED CODE */
        if(t % mfpt_frequency == 0 ) { //&& t == trials){
            pSSP->sweepedStates.clear();

            pSSP->initTransMatrix();
            double fpt_val_n = pSSP->meanFirstPassageTime(pNet->getState(tf2_starts_1[0].translation), pNet->getState(tf2_goals_1[0].translation)->id);

            it_cnt = 0;
            for (pair<double,int> it : pSSP->sweepedStates) {
                stateid[it_cnt] = it.second;
                it_cnt++;
            }

            // MFPT-VI
            for(int itrr = 1; itrr <= 2; itrr++){
                for(int state_cnt = 0; state_cnt < it_cnt; state_cnt++){
                    if((_states[stateid[state_cnt]])->type==BODY || (_states[stateid[state_cnt]])->type==START){
                        double old_val = _states[stateid[state_cnt]]->optimal_value;
                        updateStateRecordsQLearning(_states[stateid[state_cnt]]);
                        tot_val_diff += fabs(_states[stateid[state_cnt]]->optimal_value - old_val);
                    }
                    else
                        continue; //ignore goals or obstacles;
                }//for

                if(tot_val_diff == 0){
                    flag = 1;
                    break;
                }

                tot_val_diff = 0;
            }
        }

        cout << tot_val_diff << endl;
        tot_val_diff_vec.push_back(tot_val_diff);
        if(flag == 1)
            break;
    }


    // Calculating Optimal Policy at each state
    vector<mdp_state_t*>& _presentStates = pNet->mdp_states;
    for(vector<mdp_state_t*>::iterator itr=_presentStates.begin(); itr!=_presentStates.end();itr++){

        if((*itr)->type == OBSTACLE || (*itr)->type == GOAL)
            continue;

        vector<pair<int, mdp_state_t*> > nextStateValue;

        for(int j = NORTH; j < NUM_ACTIONS; j++){
            if((((*itr)->successors[j])->id != (*itr)->id) && (visited[((*itr)->successors[j])->id] != 1))
                nextStateValue.push_back(make_pair(j,(*itr)));
        }

        if (nextStateValue.empty()) break;
        std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

        pair<int, mdp_state_t*> nextState = nextStateValue[0];
        (*itr)->optimal_action = (action_t)(nextState.first);
        (*itr)->optimal_value = (nextState.second)->q_values[nextState.first];
        std::fill((*itr)->actions.begin(), (*itr)->actions.end(), false);
        (*itr)->actions[nextState.first] = true;

    }

    // Writing Val Diff values to a file
    ofstream myfile;
    myfile.open ("val_diff_dyna_mfpt.txt");

    cout << "Val Diff:" << endl;
    for(int rew_id = 0; rew_id < tot_val_diff_vec.size(); rew_id++){
        myfile << tot_val_diff_vec[rew_id];
        if(rew_id != tot_val_diff_vec.size()-1)
            myfile << ", ";
    }
    myfile << endl;
    myfile.close();


    return 0;

}

struct comp{
  bool operator() (pair<double, pair<mdp_state_t*, int>> a, pair<double, pair<mdp_state_t*, int>> b) const{
      return a.first > b.first;
  }
};

double
MDP::DYNA_PS(vector<mdp_state_t*>& _states){

    int ROWSIZE = pParams->getParamNode()["environment"]["grids"]["num_rows"].as<int>();
    int COLUMNSIZE = pParams->getParamNode()["environment"]["grids"]["num_cols"].as<int>();
    int NUM_ACTIONS = 9;
    int num_iterations = pParams->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
    int trials = pParams->getParamNode()["mdp_methods"]["num_trials"].as<unsigned int>();
    double learning_rate = pParams->getParamNode()["mdp_methods"]["learning_rate"].as<double>();
    double discount_factor = pParams->getParamNode()["mdp_methods"]["discount_factor"].as<double>();
    double action_cost = -1.0 * pParams->getParamNode()["mdp_methods"]["action_cost"].as<double>();
    double goal_reward = pParams->getParamNode()["mdp_methods"]["goal_reward"].as<double>();
    double explore_rate = pParams->getParamNode()["mdp_methods"]["explore_rate"].as<double>();


    vector<Transform2> tf2_starts_1, tf2_goals_1;
    vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
    tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
    vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
    tf2_goals_1.push_back(Transform2(vec1[0], vec1[1], vec1[2]));

    SSP::Ptr pSSP = SSP::Ptr(new SSP(pParams, pNet, pDisturb));

    // Set start and goal
    int start = -1;
    int goal = pNet->getState(tf2_goals_1[0].translation)->id;

    // Setting visited to 0
    double visited[ROWSIZE*COLUMNSIZE];
    for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
    {
        visited[i] = 0;
    }


    double tot_val_diff = 0;
    vector<double> tot_val_diff_vec;

    for (int t = 1; t <= trials; t++)
    {
        std::cout << "\n\nTrial " << t << "\n\n";

        int current = start;
        while (true)
        {
            current = rand() % (ROWSIZE*COLUMNSIZE);
            if (current != goal && _states[current]->type != OBSTACLE) break;
        }

        tot_val_diff = 0;
        priority_queue<pair<double, pair<mdp_state_t*, int>> , vector< pair<double, pair<mdp_state_t*, int>> >, comp> pq;

        for (int k = 1; k <= num_iterations; k++)
        {

            mdp_state_t* st = _states[current];
            visited[current] = 1;
            vector<pair<int, mdp_state_t*> > nextStateValue;


            for(int j = NORTH; j < NUM_ACTIONS; j++){
                if(((st->successors[j])->id != st->id) && (visited[(st->successors[j])->id] != 1))
                    nextStateValue.push_back(make_pair(j,st));
            }

            if (nextStateValue.empty()) break;
            std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

            // Exploration-Exploitation TradeOff
            pair<int, mdp_state_t*> nextState = nextStateValue[0];
            int curr_main_action;
            int p = min((rand() % nextStateValue.size()), nextStateValue.size() - 1);
            //curr_main_action = (rand() % 100 < explore_rate) ? nextStateValue[p].first : nextState.first;
            curr_main_action = nextState.first;

            // Next action
            while((st->successors[curr_main_action])->id == st->id || (st->successors[curr_main_action])->type == OBSTACLE){

                p = min((rand() % NUM_ACTIONS), NUM_ACTIONS - 1);
                curr_main_action = (rand() % 100 < explore_rate) ? p : nextState.first;

            }

            // Next State Max Value
            mdp_state_t* next_st = (st->successors[curr_main_action]);
            double value = -INF;
            for (int i = 1; i < NUM_ACTIONS; i++)
            {
                if (next_st->q_values[i] > value)
                    value = next_st->q_values[i];
            }

            // Calculate immediate reward
            double rew;
            if((st->successors[curr_main_action])->type == OBSTACLE) rew = action_cost;
            else if((st->successors[curr_main_action])->type == GOAL) rew = goal_reward;
            else rew = action_cost;

            // Priority
            double priority = fabs(rew + (discount_factor * value) - st->q_values[curr_main_action]);
            if(priority > 0)
                pq.push(make_pair(priority, make_pair(st, curr_main_action)));



            // Planning
            int planning_step = 0;
            while( planning_step <= 5 && !pq.empty()){

                pair<double, pair<mdp_state_t*, int>> plan_current = pq.top();
                pq.pop();

                mdp_state_t* plan_current_state = (plan_current.second).first;
                int plan_current_action = (plan_current).second.second;

                mdp_state_t* plan_next_state = (plan_current_state->successors[plan_current_action]);
                double value = -INF;
                for (int ii = 1; ii < NUM_ACTIONS; ii++)
                {
                    if (plan_next_state->q_values[ii] > value)
                        value = plan_next_state->q_values[ii];
                }

                double rew;
                if((plan_current_state->successors[plan_current_action])->type == OBSTACLE) rew = action_cost;
                else if((plan_current_state->successors[plan_current_action])->type == GOAL) rew = goal_reward;
                else rew = action_cost;

                double newq_valuescalc = plan_current_state->q_values[plan_current_action] + learning_rate * (rew + (discount_factor * value) - plan_current_state->q_values[plan_current_action]);
                //tot_val_diff += fabs(newq_valuescalc - _states[plan_current_state]->q_values[plan_current_action]);
                plan_current_state->q_values[plan_current_action] = newq_valuescalc;

                vector<mdp_state_t*>& _allStates = pNet->mdp_states;
                for(vector<mdp_state_t*>::iterator all_itr=_allStates.begin(); all_itr!=_allStates.end(); all_itr++){
                    for(int next_ac = 1; next_ac < NUM_ACTIONS; next_ac++){
                        mdp_state_t* prev_st = *all_itr;
                        if((prev_st->successors[next_ac])->type != OBSTACLE && (prev_st->successors[next_ac])->id != prev_st->id && (prev_st->successors[next_ac])->id == plan_current_state->id){

                            // Next State Max Value
                            double value = -INF;
                            for (int iii = 1; iii < NUM_ACTIONS; iii++)
                            {
                                if (plan_current_state->q_values[iii] > value)
                                    value = plan_current_state->q_values[iii];
                            }

                            // Calculate immediate reward
                            double rew;
                            if((prev_st->successors[next_ac])->type == OBSTACLE) rew = action_cost;
                            else if((prev_st->successors[next_ac])->type == GOAL) rew = goal_reward;
                            else rew = action_cost;

                            double priority = fabs(rew + (discount_factor * value) - prev_st->q_values[next_ac]);
                            if(priority > 0)
                                pq.push(make_pair(priority, make_pair(prev_st, next_ac)));

                        }
                    }
                }

                planning_step++;
            }

            if (next_st->id == goal)
            {
                cout << "Reached" << endl;
                break;
            }
            current = (st->successors[curr_main_action])->id;
        }


        // Reset visited to 0
        for (int i = 0; i < ROWSIZE*COLUMNSIZE; i++)
        {
            visited[i] = 0;
        }


//        cout << tot_val_diff << endl;
//        tot_val_diff_vec.push_back(tot_val_diff);

//        vector<double> y(tot_val_diff_vec.end() - 70, tot_val_diff_vec.end());
//        double sum_of_elems = 0;
//        for (auto& n : y)
//            sum_of_elems += n;

//        if (sum_of_elems/70.0 == 0)
//            break;
    }

    // Calculating Optimal Policy at each state
    vector<mdp_state_t*>& _presentStates = pNet->mdp_states;
    for(vector<mdp_state_t*>::iterator itr=_presentStates.begin(); itr!=_presentStates.end();itr++){

        if((*itr)->type == OBSTACLE || (*itr)->type == GOAL)
            continue;

        vector<pair<int, mdp_state_t*> > nextStateValue;

        for(int j = NORTH; j < NUM_ACTIONS; j++){
            if((((*itr)->successors[j])->id != (*itr)->id) && (visited[((*itr)->successors[j])->id] != 1))
                nextStateValue.push_back(make_pair(j,(*itr)));
        }

        if (nextStateValue.empty()) break;
        std::sort(nextStateValue.begin(), nextStateValue.end(), pairCompare);

        pair<int, mdp_state_t*> nextState = nextStateValue[0];
        (*itr)->optimal_action = (action_t)(nextState.first);
        (*itr)->optimal_value = (nextState.second)->q_values[nextState.first];
        std::fill((*itr)->actions.begin(), (*itr)->actions.end(), false);
        (*itr)->actions[nextState.first] = true;

    }

//    // Writing Val Diff values to a file
//    ofstream myfile;
//    myfile.open ("val_diff_dyna.txt");

//    cout << "Val Diff:" << endl;
//    for(int rew_id = 0; rew_id < tot_val_diff_vec.size(); rew_id++){
//        myfile << tot_val_diff_vec[rew_id];
//        if(rew_id != tot_val_diff_vec.size()-1)
//            myfile << ", ";
//    }
//    myfile << endl;
//    myfile.close();

    return 0;

}


void
MDP::cleanStates(void) {

    for (uint i = 0; i < pNet->mdp_states.size(); i++) {
        mdp_state_t* s = pNet->mdp_states[i];
        //if(s->spawn_parent)  s=s->spawn_parent;
        s->reset_state();
    }

}
