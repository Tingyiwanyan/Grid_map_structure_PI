
#include <iostream>
#include <fstream>
#include <ctime>

#include "spline/UnconstrainedSplineFit.h"

using namespace std;
using namespace arma;

int main (int argc, char **argv)
{

  vec knottimes, waypts, fixedderivs, fixedderivs_idxs;
  knottimes << 11.0290000000000 << 12.0090000000000 << 13.0090000000000 << 13.4890000000000;// << 13.5090000000000;
  waypts << 2.00000025630000 << 3.28643932940000 << 4.75265291960000 << 5.40911312940000;// << 5.43581299230000;
  fixedderivs << 4.95546000000000e-05 << 1.53183660560000 << 1.40002055640000 << 1.33643145860000// << 0
              << 0.00794090090000000 << -0.419731028200000 << -0.177697364500000 << -0.143914218700000;// << 0;
  fixedderivs_idxs << 1.0000 << 6.0000 << 11.0000 << 16.0000// << 21
                   << 2.0000 << 7.0000 << 12.0000 << 17.0000;// << 22;
  double sample_dt = 0.01;
  vec waypts_2 = waypts%waypts*0.5;
  
  Spline sp_x(knottimes, waypts, fixedderivs, fixedderivs_idxs, 9, sample_dt);
  Spline sp_y(knottimes, waypts_2, fixedderivs, fixedderivs_idxs, 9, sample_dt);

  int numSamples = ceil((knottimes(waypts.n_elem-1)-knottimes(0))/sample_dt);
  vec dense_path_t = linspace(knottimes(0),knottimes(waypts.n_elem-1),numSamples);

  ofstream fs;
  fs.open("UnconstrainedSplineFit_test_out.m");
  fs << "knottimes=[" << knottimes << "];" << endl
       << "waypts_x=[" << waypts << "];" << endl
       << "waypts_y=[" << waypts_2 << "];" << endl
       << "dense_path_t=[" << dense_path_t << "];" << endl
       << "dense_path_x=[" << endl << sp_x.getDensePath() << "];" << endl
       << "dense_path_y=[" << endl << sp_y.getDensePath() << "];" << endl
       << "plot(waypts_x,waypts_y,'b*', dense_path_x,dense_path_y,'r-')" << endl
       << "xlabel('x'); ylabel('y'); legend('waypoints', 'spline')" << endl
       << "figure" << endl
       << "plot(knottimes,waypts_x,'b*', dense_path_t,dense_path_x,'b-',"
       << "knottimes,waypts_y,'r*', dense_path_t,dense_path_y,'r-')" << endl
       << "xlabel('t'); legend('x waypoints', 'x spline', 'y waypoints', 'y spline')" << endl;
  fs.close();
  cout << "Saved output to UnconstrainedSplineFit_test_out.m in current directory" << endl;
  return 0;
}
