#include "mdp_methods/ssp.h"
using namespace mdp_planner;
using namespace std::chrono;

SSP::SSP(const utils::Parameters::Ptr& pParams, const MDP_Net::Ptr& pNet, const Disturbance::Ptr& pDisturb) : MDP(pParams, pNet, pDisturb) {
};

SSP::~SSP() {
};

void SSP::initTransMatrix() {
  transMatrix.resize(pNet->n_rows * pNet->n_cols, pNet->n_rows * pNet->n_cols);

  //cout << "Size: " << pNet->mdp_states.size();
  //cout << "CC: " << pNet->cell_centers[0] << endl;
  for (int i = 0; i < pNet->mdp_states.size(); ++i) {
    //mdp_state_t * state = pNet->getState(i);
    mdp_state_t * state = pNet->getState(pNet->cell_centers[i]);
    //cout << "i: " << i << " id: " <<  state->id << endl;
    int flag = 0;
    double obst_prob = 0.000001;
    for(uint obs_num_id = 0; obs_num_id < pNet->mdp_obstacle_ids.size(); obs_num_id++)
    {
        int new_id = pNet->mdp_obstacle_ids[obs_num_id];
        if(new_id == state->id){
            int state_id = state->id, cols = pNet->n_cols;
            for (uint k = ZERO; k < NUM_ACTIONS; ++k) {
              if (state->post_probs[k] != 0) {
                if (k == ZERO) {
                  transMatrix.coeffRef(state_id, state_id) = 1-(8*obst_prob);
                         //cout<<"mark1ZERO"<<endl;
                } else if (k == NORTH) {
                  transMatrix.coeffRef(state_id, state_id + cols) = obst_prob;
                          //cout<<"mark1North"<<endl;
                } else if (k == NE) {
                  transMatrix.coeffRef(state_id, state_id + cols + 1) = obst_prob;
                          //cout<<"mark1NE"<<endl;
                } else if (k == EAST) {
                  transMatrix.coeffRef(state_id, state_id + 1) = obst_prob;
                          //cout<<"EASTmark1"<<endl;
                } else if (k == SE) {
                  transMatrix.coeffRef(state_id, state_id - cols + 1) = obst_prob;
                          //cout<<"SEmark1"<<endl;
                } else if (k == SOUTH) {
                  transMatrix.coeffRef(state_id, state_id - cols) = obst_prob;
                          //cout<<"SOUTHmark1"<<endl;
                } else if (k == SW) {
                  transMatrix.coeffRef(state_id, state_id - cols - 1) = obst_prob;
                          //cout<<"SWmark1"<<endl;
                } else if (k == WEST) {
                  transMatrix.coeffRef(state_id, state_id - 1) = obst_prob;
                          //cout<<"WESTmark1"<<endl;
                } else if (k == NW) {
                  transMatrix.coeffRef(state_id, state_id + cols - 1) = obst_prob;
                          //cout<<"NWmark1"<<endl;
                }
            }
            flag = 1;
          }
        }
    }
    if(flag == 0){
        fillTransMatrix(transMatrix, state->id, pNet->n_cols, state->post_probs);
    }
    f_ij.push_back(0);
  }

  //cout << "I am here Init Trans 2" << endl;

  for (int i = 0; i < pNet->n_rows; i++) {
    for (int j = 0; j < pNet->n_cols; j++) {
      if (i == 1 && (j == 3 || j == 4)) {
        //std::cout << "transition matrix of state (" << i << ", " << j << ")" << std::endl;
        //printTransMatrix(i, j);
      }
    }
  }

}

void SSP::printTransMatrix(const int &state_i, const int &state_j) {
  //  std::cout << "transMatrix.row(0) \n" << transMatrix << endl;
  for (int i = pNet->n_rows - 1; i >= 0; i--) {
    for (int j = 0; j < pNet->n_cols; j++) {
      std::cout << transMatrix.coeffRef(state_i * pNet -> n_cols + state_j, i * pNet -> n_cols + j) << " ";
    }
    std::cout << std::endl;
  }
}

//
//probs: ZERO, N, NE, E, SE, S, SW, W, NW

void SSP::fillTransMatrix(SpMat& matrix, const int &state_id, const int &cols, const vector<double>& probs) {
  //cout << "I am here fill Trans 1: " << matrix.size() << " " << state_id << " " << cols << endl;
  double obst_prob = 0.000001;
  for (uint i = ZERO; i < NUM_ACTIONS; ++i) {
    if (probs[i] != 0) {
      if (i == ZERO) {
        matrix.coeffRef(state_id, state_id) = probs[i];
        //        cout<<"mark1ZERO"<<endl;
      } else if (i == NORTH) {
        matrix.coeffRef(state_id, state_id + cols) = probs[i];
        //if (std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), state_id + cols) != pNet->mdp_obstacle_ids.end()) {matrix.coeffRef(state_id, state_id + cols) = obst_prob;}
        //        cout<<"mark1North"<<endl;
      } else if (i == NE) {
        matrix.coeffRef(state_id, state_id + cols + 1) = probs[i];
        //if (std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), state_id + cols + 1) != pNet->mdp_obstacle_ids.end()) {matrix.coeffRef(state_id, state_id + cols + 1) = obst_prob;}
        //        cout<<"mark1NE"<<endl;
      } else if (i == EAST) {
        matrix.coeffRef(state_id, state_id + 1) = probs[i];
        //if (std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), state_id + 1) != pNet->mdp_obstacle_ids.end()) {matrix.coeffRef(state_id, state_id + 1) = obst_prob;}
        //        cout<<"EASTmark1"<<endl;
      } else if (i == SE) {
        matrix.coeffRef(state_id, state_id - cols + 1) = probs[i];
        //if (std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), state_id - cols + 1) != pNet->mdp_obstacle_ids.end()) {matrix.coeffRef(state_id, state_id - cols + 1) = obst_prob;}
        //        cout<<"SEmark1"<<endl;
      } else if (i == SOUTH) {
        matrix.coeffRef(state_id, state_id - cols) = probs[i];
        //if (std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), state_id - cols) != pNet->mdp_obstacle_ids.end()) {matrix.coeffRef(state_id, state_id - cols) = obst_prob;}
        //        cout<<"SOUTHmark1"<<endl;
      } else if (i == SW) {
        matrix.coeffRef(state_id, state_id - cols - 1) = probs[i];
        //if (std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), state_id - cols - 1) != pNet->mdp_obstacle_ids.end()) {matrix.coeffRef(state_id, state_id - cols - 1) = obst_prob;}
        //        cout<<"SWmark1"<<endl;
      } else if (i == WEST) {
        matrix.coeffRef(state_id, state_id - 1) = probs[i];
        //if (std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), state_id - 1) != pNet->mdp_obstacle_ids.end()) {matrix.coeffRef(state_id, state_id - 1) = obst_prob;}
        //        cout<<"WESTmark1"<<endl;
      } else if (i == NW) {
        matrix.coeffRef(state_id, state_id + cols - 1) = probs[i];
        //if (std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), state_id + cols - 1) != pNet->mdp_obstacle_ids.end()) {matrix.coeffRef(state_id, state_id + cols - 1) = obst_prob;}
        //        cout<<"NWmark1"<<endl;
      }
    }
  }
}

void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove) {
  unsigned int numRows = matrix.rows() - 1;
  unsigned int numCols = matrix.cols();

  if (rowToRemove < numRows)
    matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.block(rowToRemove + 1, 0, numRows - rowToRemove, numCols);

  matrix.conservativeResize(numRows, numCols);
}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove) {
  unsigned int numRows = matrix.rows();
  unsigned int numCols = matrix.cols() - 1;

  if (colToRemove < numCols)
    matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.block(0, colToRemove + 1, numRows, numCols - colToRemove);

  matrix.conservativeResize(numRows, numCols);
}

//bool SSP::pairCompare(const std::pair<int, double>& firstElem, const std::pair<int, double>& secondElem) {
//  return firstElem.second < secondElem.second;

//}

double SSP::meanFirstPassageTime(const mdp_state_t* start_state, const int &end_state) {
  double states_fpt[pNet -> n_rows * pNet -> n_cols];
  // \mu_{sj} = 1 + \sum_k_{k} p_{sk} \mu_{kj}, where \mu_{ii} = 0;
  // \sum_k_{k} p_{sk} \mu_{kj} - \mu_{sj} = -1;
  // => Ax = b 
  // x = \mu_{1j}, \dots, \mu_{nj} column vector (n X 1)
  // b = -1 column vector (n X 1)
  // A = coefficient matrix (n X n)

  // Test
  //  Eigen::MatrixXd transMatrix(4, 4);
  //  transMatrix << 0.080, 0.184, 0.368, 0.368,
  //          0.632, 0.368, 0, 0,
  //          0.264, 0.368, 0.368, 0,
  //          0.080, 0.184, 0.368, 0.368;
  //  std::cout << transMatrix;
  //

  int n = transMatrix.rows();
  //std::cout << "N: " << n << std::endl;
  Eigen::MatrixXd I(n - 1, n - 1);
  I.setIdentity();
  Eigen::SparseMatrix<double> full_A = transMatrix;

  //  std::cout << "full_A " << full_A;

  Eigen::VectorXd b = -Eigen::VectorXd::Ones(n - 1);
  //  std::cout << "b " << std::endl;
  //  std::cout << b << std::endl;

  // solve Ax = b
  //  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  Eigen::SimplicialLDLT<Eigen::MatrixXd> solver;

  vector<double> mu_s(n, -1);

  // meanFirstPassageTime to state i
  for (int i = end_state; i <= end_state; ++i) {
    Eigen::MatrixXd temp_A = full_A;

    // my code trial

    //std::cout << pNet->mdp_obstacle_ids.size() << std::endl;
    /*for(uint obs_num_id = 0; obs_num_id < pNet->mdp_obstacle_ids.size(); obs_num_id++)
    {
        int new_id = pNet->mdp_obstacle_ids[obs_num_id];
        temp_A.row(new_id).setZero();
        temp_A(new_id,new_id) = 1;
        Eigen::MatrixXd tt = -0.000001 * Eigen::MatrixXd::Ones(n,n);
        temp_A.row(new_id) += tt.row(new_id);
    }*/
//    std::cout << temp_A.row(525) << std::endl;
//    temp_A.row(525).setZero();
//    std::cout << "New" << std::endl;
//    temp_A(525,525) = 1;
//    Eigen::MatrixXd tt = -0.00000001 * Eigen::MatrixXd::Ones(n,n);
//    temp_A.row(525) += tt.row(525);
//    std::cout << temp_A.row(525) << std::endl;

//    std::cout << temp_A.row(475) << std::endl;
//    temp_A.row(475).setZero();
//    std::cout << "New" << std::endl;
//    temp_A(475,475) = 1;
//    Eigen::MatrixXd tt1 = -0.00000001 * Eigen::MatrixXd::Ones(n,n);
//    temp_A.row(475) += tt1.row(475);
//    std::cout << temp_A.row(475) << std::endl;

    removeColumn(temp_A, i);
    removeRow(temp_A, i);
    temp_A = temp_A - I;
    //    std::cout << "temp_A" << std::endl;
    //    std::cout << temp_A << std::endl;

    Eigen::MatrixXd A = temp_A;

    //    Eigen::SparseMatrix<double> A = temp_A.sparseView();
    //    solver.compute(A);
    //    if (solver.info() != Eigen::Success) {
    //      std::cerr << "decomposition failed " << std::endl;
    //    }
    //    Eigen::VectorXd x = solver.solve(b);

    Eigen::VectorXd x = A.partialPivLu().solve(b);

    //    if (solver.info() != Eigen::Success) {
    //      std::cerr << "solving failed" << std::endl;
    //    }
    //    std::cout << "x " << std::endl;
    //    std::cout << x << std::endl;


    // UNCOMMENT TO DISPLAY VALUES

    //std::cout << "meanFirstPassageTime to state i " << i << std::endl;
    for (int j = pNet -> n_rows - 1; j >= 0; j--) {
      for (int k = 0; k < pNet -> n_cols; k++) {
        int index = j * pNet -> n_cols + k;
        if (index == i) {
          states_fpt[index] = 0;
          //std::cout << "0 ";
        } else if (index < i) {
          states_fpt[index] = fabs(x[j * pNet -> n_cols + k]);
          //if(fabs(x[j * pNet -> n_cols + k]) < 600) std::cout << fabs(x[j * pNet -> n_cols + k]) << " "; else std::cout << "600 ";

          if(fabs(x[j * pNet -> n_cols + k]) > 500) { pNet->reachablestates.push_back(index); } else if(fabs(x[j * pNet -> n_cols + k]) > 0 && fabs(x[j * pNet -> n_cols + k]) < 500) { pNet->reachablestatesdg.push_back(index); }
        } else {
          states_fpt[index] = fabs(x[j * pNet -> n_cols + k - 1]);
          //if(fabs(x[j * pNet -> n_cols + k - 1]) < 600) std::cout << fabs(x[j * pNet -> n_cols + k - 1]) << " "; else std::cout << "600 ";

          if(fabs(x[j * pNet -> n_cols + k - 1]) > 500) { pNet->reachablestates.push_back(index); } else if(fabs(x[j * pNet -> n_cols + k - 1]) > 0 && fabs(x[j * pNet -> n_cols + k - 1]) < 500) { pNet->reachablestatesdg.push_back(index); }
        }

      }
      //std::cout << ";" << std::endl;
    }
    //std::cout << "**********************" << std::endl;

    // check
//    if (!(A * x).isApprox(b, 1e-7)) {
//      std::cout << "Ax - b" << std::endl;
//      //std::cout << A * x - b << std::endl;
//    }

    if (start_state->id == i) {
      mu_s[i] = 0;
    } else if (start_state->id < i) {
      mu_s[i] = x[start_state->id];
    } else {
      mu_s[i] = x[start_state->id - 1];
    }
  }

//    std::cout << "Mean First Passage Time from state " << start_state->id << std::endl;
//    for (int i = pGrid2d -> n_rows - 1; i >= 0; i--) {
//      for (int j = 0; j < pGrid2d -> n_cols; j++) {
//        std::cout << mu_s[i * pGrid2d -> n_cols + j] << " ";
//      }
//      std::cout << std::endl;
//    }

   //UNCOMMENT TO DISPLAY VALUES
//  std::cout << "Mean First Passage Time from state " << start_state->id << " to state " << end_state << std::endl;
//  std::cout << fabs(mu_s[end_state]) << std::endl;



  // Place to comment/uncomment
  std::cout.precision(3);

  double states_fpt_smooth[pNet -> n_rows * pNet -> n_cols];

  for(int iter = 1; iter <= 1; iter++){
      for (int i = 0; i < pNet -> n_rows * pNet -> n_cols; ++i) {
          //if(i % pNet->n_cols == 0) cout << endl;
          int succ_index = -1;
          mdp_state_t * state = pNet->getState(pNet->cell_centers[i]);

          if(i != end_state && std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), i) == pNet->mdp_obstacle_ids.end())// && i < 480)
          {
              double sum = 0;
              double count = 0;
              double min_val = -1;
              for(int j = 1; j<=8; j++)
              {
                  succ_index = state->successors[(action_t)j]->id;
                  if ((std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), succ_index) != pNet->mdp_obstacle_ids.end()))// || (succ_index == end_state))
                      continue;
                  else
                  {
                      /*if(states_fpt[succ_index] < 500) { sum += 0.1*states_fpt[succ_index]; count += 0.1;}
                      else {
                        sum += states_fpt[succ_index];
                        count++;
                      }*/
                      sum += states_fpt[succ_index];
                      count++;

                      if(min_val == -1) min_val = states_fpt[succ_index];
                      else if(min_val > states_fpt[succ_index]) min_val = states_fpt[succ_index];
                  }
              }
              states_fpt_smooth[i] = (sum/count);
              //states_fpt_smooth[i] = min_val;
          }
          else
          {
              //cout << "state: " << i << endl;
              states_fpt_smooth[i] = states_fpt[i];
          }
          //if(states_fpt_smooth[i] < 100) cout << states_fpt_smooth[i] << " "; else cout << 100 << " ";
      }

      for (int i = 0; i < pNet -> n_rows * pNet -> n_cols; ++i) {
          states_fpt[i] = states_fpt_smooth[i];
      }
  }


  // Keep it commented
  /*cout << "\n\n Further Smoothing \n\n" << endl;
  double states_fpt_smooth_2[pNet -> n_rows * pNet -> n_cols];
  for (int i = 0; i < pNet -> n_rows; ++i) {
      int start_high = -1, end_high = -1;
      int start_low = -1, end_low = -1;
      for (int j = 0; j < pNet -> n_cols; ++j) {
        int state_id = (i * pNet->n_cols) + j;

        //if(std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), state_id) == pNet->mdp_obstacle_ids.end()){
        if(i != 15){
            if(states_fpt_smooth[state_id] < 100){
                if(start_high != -1) end_high = state_id - 1;
                states_fpt_smooth_2[state_id] = states_fpt_smooth[state_id];


                if(start_high -1 >= i * pNet->n_cols) start_low = start_high -1;
                if(end_high != -1) end_low = (end_high + 1 < i * pNet->n_cols + pNet->n_cols) ? end_high + 1 : -1;

                if(start_high != -1 && end_high != -1){
                    //cout << "\n\n" << start_high << " " << end_high << " " << start_low << " " << end_low << " ";
                    double new_val = -1.0;
                    if(start_low != -1) new_val = states_fpt_smooth[start_low];
                    if(end_low != -1) new_val = states_fpt_smooth[end_low];
                    if(start_low != -1 && end_low != - 1) new_val = (states_fpt_smooth[start_low] + states_fpt_smooth[end_low])/2;

                    //cout << new_val << "\n\n";
                    for(int k = start_high; k <= end_high; ++k){
                        states_fpt_smooth_2[k] = new_val;
                        cout << states_fpt_smooth_2[k] << " ";
                    }
                    start_high = -1, end_high = -1, start_low = -1, end_low = -1;
                }
                cout << states_fpt_smooth_2[state_id] << " ";
            }

            if(states_fpt_smooth[state_id] > 100){
                if(start_high == -1) start_high = state_id;

                if(state_id == (i+1)*pNet -> n_cols - 1){
                    end_high = state_id;
                    double new_val = -1.0;
                    //cout << "\n\n" << start_high << " " << end_high << " " << start_low << " " << end_low << "\n\n";
                    new_val = states_fpt_smooth[start_high-1];
                    for(int k = start_high; k <= end_high; ++k){
                        states_fpt_smooth_2[k] = new_val;
                        cout << states_fpt_smooth_2[k] << " ";
                    }

                }

            }

          }
          else {
            states_fpt_smooth_2[state_id] = states_fpt_smooth[state_id];
            cout << (states_fpt_smooth_2[state_id] > 100 ? 100 : states_fpt_smooth_2[state_id])  << " ";

        }



      }
      cout << endl;
  }*/

//  cout << "\n\n Displaying Smooth Values \n\n";
//  for (int j = pNet -> n_rows - 1; j >= 0; j--) {
//    for (int k = 0; k < pNet -> n_cols; k++) {
//      int index = j * pNet -> n_cols + k;
//      cout << (states_fpt_smooth[index] > 100 ? 100 : states_fpt_smooth[index]) << " ";
//    }
//    cout << endl;
//  }

  for (int i = 0; i < pNet -> n_rows * pNet -> n_cols; ++i) {
      states_fpt[i] = states_fpt_smooth[i];
      sweepedStates.push_back(( std::pair<double,int>(states_fpt[i],i) ));
  }
  sort(sweepedStates.begin(), sweepedStates.end());

  int isSubSpaceAnalysis = pParams->getParamNode()["environment"]["isSubSpace"].as<int>();

  int loopstartindex, loopendindex, loopindex;
  if(isSubSpaceAnalysis == 1){

      /*  UNCOMMENT THIS FOR SUB SPACE */
      int numstates = pNet -> n_rows * pNet -> n_cols;
      int tot_partitions = pParams->getParamNode()["environment"]["partitions"]["num_partitions"].as<int>();  // CHANGE IT WHEN NUMBER OF PARTITIONS NEED TO BE UPDATED
      loopindex = loopind;
      cout << "Loop: " << loopindex << endl;
      if(loopindex == 1) {loopstartindex = 0; loopendindex = numstates;}
      else {

          /* UNCOMMENT FOR H1 */
          if(pParams->getParamNode()["mdp_methods"]["iteration_method"].as<string>().compare("D-MFPT-VI-H1-Sub") || pParams->getParamNode()["mdp_methods"]["iteration_method"].as<string>().compare("MFPT-VI-Sub")){
              loopstartindex = ((loopindex-1)*numstates)/tot_partitions;
              //loopstartindex = 0;
              loopendindex = loopindex*numstates/tot_partitions;
          }

          /* COMMENT OR UNCOMMENT FOR H2 */
          if(pParams->getParamNode()["mdp_methods"]["iteration_method"].as<string>().compare("D-MFPT-VI-H2-Sub")){
              double e_min = (sweepedStates[0]).first, e_max = (sweepedStates[sweepedStates.size() - 20]).first;
              double intv = (e_max - e_min)/tot_partitions;
              double start_value = e_min + ((loopindex-1)*intv), end_value = e_min + (loopindex*intv);
              cout << start_value << " : " << end_value << endl;

              loopstartindex = 0, loopendindex = 0;
              for (pair<double,int> it : sweepedStates) {
                if(it.first > start_value)
                {
                    break;
                }
                loopstartindex++;
              }
              for (pair<double,int> it : sweepedStates) {
                if(it.first > end_value)
                {
                    break;
                }
                loopendindex++;
              }
              //loopstartindex--;
              //loopendindex--;
          }
      }

      cout << "Loop Start: " << loopstartindex << ", Loop End: " << loopendindex << endl;
  }

  /* CHANGE THE FOR LOOP FOR SUB SPACE CALCULATION */

  if( isSubSpaceAnalysis == 0){
      loopstartindex = 0;
      loopendindex = pNet -> n_rows * pNet -> n_cols;
  }
  //for (int i = 0; i < pNet -> n_rows * pNet -> n_cols; ++i) {
  for (int i = loopstartindex; i < loopendindex; ++i) {
    //cout << "i: " << i << endl;
    //mdp_state_t * state = pNet->getState(i);

    mdp_state_t * state;

    /* UNCOMMENT FOR FULL SPACE */

    if( isSubSpaceAnalysis == 0){ state = pNet->getState(pNet->cell_centers[i]);}

    /* UNCOMMENT THIS FOR SUB SPACE */
    if( isSubSpaceAnalysis == 1){
          if(loopindex == 1) { state = pNet->getState(pNet->cell_centers[i]); }
        else { state = pNet->getState(pNet->cell_centers[sweepedStates[i].second]); }
    }

    std::fill(state->actions.begin(), state->actions.end(), false);

    //int succ_index = state->successors[NORTH]->id;
    //int min_val = states_fpt[succ_index], min_action = 1;

    int succ_index = -1;
    int min_val = -1, min_action = -1;

    for(int j = 1; j<=8; j++){
        succ_index = state->successors[(action_t)j]->id;
        //cout << "Hi " << j << endl;
        if (std::find(pNet->mdp_obstacle_ids.begin(), pNet->mdp_obstacle_ids.end(), succ_index) != pNet->mdp_obstacle_ids.end())
            continue;

        if(min_val == -1 && min_action == -1)
        {
            min_val = states_fpt[succ_index];
            min_action = j;
            continue;
        }

        if(succ_index == end_state)
        {
            min_val = states_fpt[succ_index];
            min_action = j;
            break;
        }
        if(states_fpt[succ_index] <= min_val && states_fpt[succ_index] != 0){
            min_val = states_fpt[succ_index];
            min_action = j;
        }
        //std::cout << succ_index << ", " << states_fpt[succ_index] << std::endl;

    }

    // RECENTLY CHNAGED [UNCOMMENT WHEN NEEDED TO UPDATE POLICY]
//    if(min_action!= -1){
//        state->optimal_action = (action_t)min_action;
//        state->actions[(action_t)min_action] = true;
//        //std::cout << state->id << ", " << state->optimal_action << std::endl;
//    }

    for(int k=0; k < pNet->mdp_obstacle_ids.size(); k++){
        mdp_state_t * state = pNet->getState(pNet->cell_centers[pNet->mdp_obstacle_ids[k]]);
        std::fill(state->actions.begin(), state->actions.end(), false);
    }
    //std::cout << std::endl;
    //std::cout << state->successors[NORTH]->id << ", "<< state->successors[NE]->id << ", "<< state->successors[EAST]->id << ", "<< state->successors[SE]->id << ", "
             // << state->successors[SOUTH]->id << ", "<< state->successors[SW]->id << ", "<< state->successors[WEST]->id << ", "<< state->successors[NW]->id << ", " << std::endl;
  }
  //cout << "SSP Calculated Inside" << endl;
  return fabs(mu_s[end_state]);
}

double SSP::mcFirstPassageTime(const int &start_state, const int &end_state) {
  if (start_state == end_state) {
    return 0;
  }

  const int iterations = 1000;
  int n = transMatrix.rows();

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);

  std::vector<std::discrete_distribution<int>> distributions(n);
  for (int i = 0; i < n; i++) {
    std::vector<double> trans(n);
    for (int j = 0; j < n; j++) {
      trans[j] = transMatrix.coeffRef(i, j);
    }
    std::discrete_distribution<int> distribution(trans.begin(), trans.end());
    distributions[i] = distribution;
  }

  double mean_steps = 0;
  if (end_state == 10 || end_state == 6) {
    std::cout << std::endl;
  }
  for (int i = 0; i < iterations; i++) {
    //    std::cout << "i " << i << std::endl;
    int steps = 0;
    int current_state = start_state;
    if (end_state == 10 || end_state == 6) {
      std::cout << current_state;
    }
    while (true) {
      // Transition
      current_state = distributions[current_state](generator);
      steps++;
      //      std::cout << "current_state " << current_state << " steps " << steps << std::endl;
      if (end_state == 10 || end_state == 6) {
        std::cout << " -> " << current_state;
      }
      if (current_state == end_state) {
        //        std::cout << "end_state " << end_state << " steps " << steps << std::endl;
        if (end_state == 10 || end_state == 6) {
          std::cout << " steps:" << steps;
        }
        break;
      }
    }
    if (end_state == 10 || end_state == 6) {
      std::cout << std::endl;
    }
    mean_steps += steps;
  }
  if (end_state == 10 || end_state == 6) {
    std::cout << "mean_steps " << mean_steps / iterations << std::endl;
  }
  return mean_steps / iterations;
}

vector<double> SSP::getBel(Eigen::SparseMatrix<double> belief) {
  vector<double> rst;
  for (int i = 0; i < belief.size(); ++i) {
    rst.push_back(belief.coeffRef(0, i));
  }
  return rst;
}

vector<vector<double> > SSP::getBels() {
  return this->bels;
}

vector<mdp_state_t*>
SSP::getExpectedWayStates(mdp_state_t* _start, MDP_Net::Ptr& pNet) {

  vector<mdp_state_t*> wp;
  mdp_state_t* s = _start;
  int cnt = 0;
  do {
    wp.push_back(s);
    s = s->successors[s->optimal_action];
    assert(cnt <= pNet->mdp_states.size());
    if (cnt++ >= pNet->mdp_states.size()) break; //in case assert doesnt work
  } while (s->type != GOAL);
  //finally add the last one -- goal
  wp.push_back(s);

  return wp;

}

vector<mdp_state_t*>
SSP::getExpectedWayStates(mdp_state_t* _start) {

  return getExpectedWayStates(_start, this->pNet);

}

vector<mdp_state_t*>
SSP::getExpectedWayStates(mdp_state_t* _start, mdp_state_t* _goal, MDP_Net::Ptr& pNet) {

  vector<mdp_state_t*> wp;
  mdp_state_t* s = _start;
  int cnt = 0;
  //do{
  while (s->type != GOAL && cnt < pNet->mdp_states.size()) {
    wp.push_back(s);
    //r->waystates.push_back(r->state->successors[r->state->m_values[g].second]);
    //s = s->successors[s->optimal_action];
    //assert(s->m_values.size() && s->m_values[_goal]);
    s = s->successors[s->m_values[_goal].second];
    cnt++;
  }
  //finally add the absorbing state -- goal
  wp.push_back(_goal);

  //if failed, return void
  if (cnt == pNet->mdp_states.size()) {
    cout << "Geting path from starting state# " << _start->id << " failed." << endl;
    wp.clear();
  }

  return wp;

}

vector<mdp_state_t*>
SSP::getExpectedWayStates(mdp_state_t* _start, mdp_state_t* _goal) {

  return getExpectedWayStates(_start, _goal, this->pNet);

}

vector<Transform2>
SSP::getWaypoints(const vector<mdp_state_t*>& _waystates, MDP_Net::Ptr& pNet) {

  vector<Transform2> tfs;
  for (uint i = 0; i < _waystates.size(); i++) {
    mdp_state_t* s = _waystates[i];
    Transform2 _tf2(pNet->cell_centers[s->id]);
    tfs.push_back(_tf2);
  }

  return tfs;
}

vector<Transform2>
SSP::getWaypoints(const vector<mdp_state_t*>& _waystates) {

  return getWaypoints(_waystates, this->pNet);

}

vector<Transform2>
SSP::pathSharpTurnsRemoval(const vector<Transform2>& _in_paths, int _angle_threshold, int _num_nodes) {

  if (_in_paths.size() <= 2)
    return _in_paths;

  if (_num_nodes > _in_paths.size() / 2 - 1)
    _num_nodes = _in_paths.size() / 2 - 1;

  vector<Transform2> out_paths;
  vector<bool> nodes_votes(_in_paths.size(), true);
  //vector</>

  int rm_before = _num_nodes / 2;
  int rm_after = _num_nodes - rm_before;

  // so from i=1, to i=size-1, to compute included angles, 
  // since v1.v2 = |v1|*|v2| cos(theta)
  for (uint i = 1; i < _in_paths.size() - 1; i++) {
    //{{code}} below computes the included angle theta
    int theta = 0;
    {
      {
        point2d_t p = _in_paths[i].translation;
        point2d_t p_b = _in_paths[i - 1].translation;
        point2d_t p_a = _in_paths[i + 1].translation;
        point2d_t v1 = p_b - p;
        point2d_t v2 = p_a - p;
        //cos theta value
        double value = v1.dot(v2) / (v1.norm() * v2.norm());
        if (value < -1.0) value = -1.0; //saturate for safe acos
        if (value > 1.0) value = 1.0;
        theta = acos(value)*180.0 / M_PI;
      }
    }

    assert(_angle_threshold >= 0 && _angle_threshold <= 180);
    if (theta <= _angle_threshold) {
      nodes_votes[i] = false;
      //then remove nearby nodes before it
      for (uint j = 0; j < rm_before; j++) {
        if (i - j < 1) break;
        nodes_votes[i - j] = false;
      }//for j
      //then remove nearby nodes after it
      for (uint j = 0; j < rm_after; j++) {
        if (i + j > _in_paths.size() - 1) break;
        nodes_votes[i + j] = false;
      }//for j
    }//if theta
  }//for i

  //always keep the start and goal nodes of a path, they should never be removed
  nodes_votes.front() = true;
  nodes_votes.back() = true;

  for (uint i = 0; i < nodes_votes.size(); i++) {
    if (nodes_votes[i])
      out_paths.push_back(_in_paths[i]);
  }

  return out_paths;
}

vector<arma::vec>
SSP::splinePaths(const vector<Transform2>& _wp) {

  vector<arma::vec> spline_paths(2); //2 dimension
  char axis_labels[2] = {'x', 'y'};
  double sample_dt = 0.5; //move to yaml if necessary
  double speed = 0.5;

  vector<double> knots = getKnotTimes(_wp, speed);

  for (uint j = 0; j<sizeof (axis_labels) / sizeof (char); j++) {
    //cout<<"splining "<<axis_labels[j]<<endl;
    vector<double> vec_1d = getWaypoints1D(_wp, axis_labels[j]);
    assert(vec_1d.size() == knots.size());
    //std::copy(vec_1d.begin(), vec_1d.end(), std::ostream_iterator<double>(std::cout, " "));
    arma::vec knottimes, waypts; //fixedderivs, fixedderivs_idxs; 
    knottimes = arma::conv_to<arma::vec>::from(knots);
    waypts = arma::conv_to<arma::vec>::from(vec_1d);
    //Spline sp(knottimes, waypts, fixedderivs, fixedderivs_idxs, 9, sample_dt);
    Spline sp(knottimes, waypts, arma::vec(), arma::vec(), 9, sample_dt);
    spline_paths[j] = sp.getDensePath();
  }//for j

  return spline_paths;
}

vector<Transform2>
SSP::convertSplinePaths2Waypoints(const vector<arma::vec>& _paths) {

  //assert(_paths.size() >= 3);
  assert(_paths[0].n_elem == _paths[1].n_elem);

  vector<Transform2> waypts(_paths.front().n_elem);
  for (uint i = 0; i < waypts.size(); i++) {
    waypts[i] = Transform2(point2d_t(_paths[0][i], _paths[1][i]));
  }

  return waypts;
}

vector<double>
SSP::getWaypoints1D(const vector<Transform2>& _wp, char _axis_label) {

  vector<double> vec;
  for (uint i = 0; i < _wp.size(); i++) {
    Transform2 _tf2 = _wp[i];
    double value;
    switch (_axis_label) {
      case 'x': value = _tf2.translation.x();
        break;
      case 'y': value = _tf2.translation.y();
        break;
        //case 'z': value = _tf2.translation.z(); break;
      default: assert(0);
    }
    vec.push_back(value);
  }

  assert(vec.size() == _wp.size());
  return vec;
}

vector<double>
SSP::getKnotTimes(const vector<Transform2>& _waypoints, double _speed) {

  vector<double> knots;
  double time_sum = 0;
  if (!_waypoints.empty()) {
    Transform2 p1 = _waypoints.front();
    for (uint i = 0; i < _waypoints.size(); i++) {
      Transform2 p2 = _waypoints[i];
      double time = (p1.translation - p2.translation).norm() / _speed;
      time_sum += time;
      knots.push_back(time_sum);
      p1 = p2;
    }
  }
  return knots;
}
