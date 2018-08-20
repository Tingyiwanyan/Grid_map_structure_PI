#include <iostream>
#include <ctime>
#include <math.h>
#include <unistd.h>

#include "spline/UnconstrainedSplineFit.h"


using namespace arma;
using namespace std;


Spline::Spline(vec knotts, vec wpts, vec fixed_d, vec fixed_d_idxs, double order, double sample_dt)
{
  knottimes = knotts;
  waypoints = wpts;
  fixed_deriv = fixed_d;
  fixed_deriv_idxs = fixed_d_idxs;
  N = order+1;
  numWpts = waypoints.n_elem;

  fitSplineUnc();

  setDensePath(sample_dt);
}

Spline::Spline(vec coefficients, vec knotts, double order, double sample_dt)
{
  coeffs = coefficients;
  knottimes = knotts;
  N = order+1;
  numWpts = knotts.n_elem;

  setDensePath(sample_dt);
}


// Based on the paper Polynomial Trajectory Planning for Quadrotor Flight by Richter, Bry, and Roy
void Spline::fitSplineUnc()
{
  // Block diagonal matrices containing the Q and A matrices for each segment
  mat Qjoint = zeros(N*(numWpts-1),N*(numWpts-1));
  mat Ajoint = zeros(N*(numWpts-1),N*(numWpts-1));

  // Selector matrix
  mat M = zeros(N/2*numWpts, N*(numWpts-1));

  // Indicies for initial positions in the fixed derivatives vector
  vec countN2 = linspace(0,numWpts-1,numWpts)*N/2;

  // Construct Q and A matrices for each segment between waypoints
  for (int ii=0; ii<numWpts-1; ii++)
  {
    double T = knottimes(ii+1)-knottimes(ii);

    // Set up cost function components
    mat Q = zeros(N,N);
    int r = N/2-1;
    vec m = linspace(0,r-1,r);
    for (int i=r; i<N; i++)
      for (int l=r; l<N; l++)
        Q(i,l) = 2*prod((i-m)%(l-m))*pow(T,(i+l-2*r+1))/(i+l-2*r+1);

      mat A0 = zeros(N/2,N);
    mat AT = zeros(N/2,N);
    for (int r=0; r<N/2; r++)
    {
      vec m = linspace(0,r,r+1);	// using a 2-line version here to handle the r=0 case
      m.shed_row(m.n_elem-1);
      A0(r,r) = prod(r-m);
      for (int n=r; n<N; n++)
        AT(r,n) = prod(n-m)*pow(T,n-r);	//NOTE: typo in the original paper says r-m instead of n-m
    }
    mat A = join_cols(A0, AT);

    Qjoint.submat(ii*N, ii*N, ii*N+N-1, ii*N+N-1) = Q;
    Ajoint.submat(ii*N, ii*N, ii*N+N-1, ii*N+N-1) = A;

    // Populate selector matrix (re-ordered below)
    M.submat(N/2*ii,ii*N, N/2*ii+N-1, ii*N+N-1) += eye(N,N);
  }

  // Indicies of all fixed coefficients
  vec fixed_idxs = join_cols(countN2, fixed_deriv_idxs);

  // Create selector matrix for free derivatives
  mat eyeN2 = eye(numWpts*N/2,numWpts*N/2);
  mat Mp;
  for (int i=0; i<numWpts*N/2; i++)
  {
    if (sum(i==fixed_idxs)==0)
    {
      if (Mp.n_elem==0)
        Mp = eyeN2.col(i);
      else
        Mp = join_rows(Mp,eyeN2.col(i));
    }
  }

  // Create selector matrix for fixed derivatives
  mat Mf = zeros(numWpts*N/2,fixed_idxs.n_elem);
  for (unsigned int i=0; i<fixed_idxs.n_elem; i++)
    Mf(fixed_idxs(i),i) = 1;

  // Combine Mf and Mp and reorder M to group fixed and free parts
    M = join_rows(Mf, Mp).t() * M;

    // Get augmented cost function
    mat AinvMt = solve(Ajoint,M.t());
    mat R = AinvMt.t() * Qjoint * AinvMt;


    // Set values for fixed derivatives
    vec Df = join_cols(waypoints, fixed_deriv);

    // Compute optimal values for free derivatives by differentiating R
    mat Rpp = R.submat(fixed_idxs.n_elem,fixed_idxs.n_elem, R.n_rows-1,R.n_cols-1);
    mat Rfp = R.submat(0,fixed_idxs.n_elem, fixed_idxs.n_elem-1,R.n_cols-1);
    vec Dp = -solve(Rpp,Rfp.t()*Df);

    // Back out coefficients
    coeffs = AinvMt*join_cols(Df, Dp);
}


vec Spline::getCoeffs()
{
  return coeffs;
}


vec Spline::getKnotTimes()
{
  return knottimes;
}

int Spline::getNumWpts()
{
  return numWpts;
}


int Spline::getSegmentIdx(double t)
{
  if (numWpts<2)
    return -1;

  uvec::fixed<1> idx;
  // If the specified time is outside the spline's time range, use the nearest endpoint, otherwise find the best segment index
  if (t < knottimes(0))
    idx(0) = 0;
  else if (t > knottimes(knottimes.n_elem-1))
    idx(0) = knottimes.n_elem-1;
  else
    idx = find(knottimes<=t,1,"last");

  return idx(0);
}

double Spline::evalAtTime(double t, int deriv)
{
  uvec::fixed<1> idx;
  double t_;
  // If the specified time is outside the spline's time range, use the nearest endpoint, otherwise find the best segment index
  if (t < knottimes(0))
  {
    idx(0) = 0;
    t_ = 0;
  }
  else if (t > knottimes(knottimes.n_elem-1))
  {
    idx(0) = knottimes.n_elem-1;
    t_ = knottimes(knottimes.n_elem-1)-knottimes(knottimes.n_elem-2);
  }
  else
  {
    idx = find(knottimes<=t,1,"last");
    t_ = t-knottimes(idx(0));
  }

  // The last knot point is part of the segment starting at the 2nd to last knot point
  if (idx(0)==knottimes.n_elem-1)
  {
    idx = idx-1;
    t_ = knottimes(knottimes.n_elem-1)-knottimes(knottimes.n_elem-2);
  }

  double value = 0;
  for (int n=(N-1); n>=0; n--)
  {
    double k = 1;	// Compute the additional coefficient introduced by differentiation
    for (int j=n; j>n-deriv; j--)
      k *= j;

    value += k*as_scalar(pow(t_,n-deriv)*coeffs(idx*N + n));
  }

  return value;
}


void Spline::setDensePath(double sample_dt)
{
  int numSamples = ceil((knottimes(numWpts-1)-knottimes(0))/sample_dt);
  densePathTimes = linspace(knottimes(0),knottimes(numWpts-1),numSamples);
  densePath = zeros(numSamples);
  for (int j=0; j<numSamples; j++)
  {
    densePath(j) = evalAtTime(densePathTimes(j));
  }
}


vec Spline::getDensePath()
{
  return densePath;
}


// bool Spline::projectPointOnSplines(vec p, vector<Spline> splineVec, double &tout)
// {
//   if (p.n_elem != splineVec.size())
//   {
//     cerr << "dimension mismatch in projectPointOnSplines" << endl;
//     return 0;
//   }
// 
//   // Concatenate sampled splines into one array (each column is a point in n dimensions)
//   mat densePathArray = zeros(p.n_elem,100);	//TODO: replace 1000 after updating setDensePath
//   for (int i = 0; i<p.n_elem; i++)
//     densePathArray.row(i) = splineVec[i].getDensePath().t();
// 
//   // Subtract the test point from each spline point and compute 2-norm
//     mat distArray = densePathArray;
//     distArray.each_col() -= p;
//     vec distsToPoint = sum(square(distArray),0).t();
// 
//     // Find the index for the min distance
//     uvec idxs = sort_index(distsToPoint);
//     double min_idx = idxs(0);
// 
//     // Get corresponding time
//     tout = splineVec[0].densePathTimes(min_idx);
// 
//     return 1;
// }



// Spline Spline::quinticConnector(vec X0, double t0, double acc_max)
// {
//   double x0 = X0(0);
//   double v0 = X0(1);
//   double a0 = X0(2);
// 
//   double tf = 1e-6;
//   double ub = knottimes(knottimes.n_elem-1)-t0;
//   double lb = 0;
// 
//   vec6 c;
//   c(0) = x0;
//   c(1) = v0;
//   c(2) = a0/2;
//   // int numiters = 0;
//   while (ub-tf > 1e-4)
//   {
//     // 		numiters++;
//     double tf2 = tf*tf;
//     double tf3 = tf2*tf;
//     double tf4 = tf3*tf;
//     double tf5 = tf4*tf;
// 
//     double xf = evalAtTime(tf+t0);
//     double vf = evalAtTime(tf+t0,1);
//     double af = evalAtTime(tf+t0,2);
// 
//     c(3) = -(((3*a0)/2 - af/2)*tf2 + (6*v0 + 4*vf)*tf + 10*x0 - 10*xf)/tf3;
//     c(4) = (((3*a0)/2 - af)*tf2 + (8*v0 + 7*vf)*tf + 15*x0 - 15*xf)/tf4;
//     c(5) = -((a0/2 - af/2)*tf2 + (3*v0 + 3*vf)*tf + 6*x0 - 6*xf)/tf5;
// 
//     vec2 t_crit;
//     t_crit(0) = (-24*c(4) + sqrt(pow(24*c(4),2) - 4*60*c(5)*6*c(3)))/(2*60*c(5));
//     t_crit(1) = (-24*c(4) - sqrt(pow(24*c(4),2) - 4*60*c(5)*6*c(3)))/(2*60*c(5));
// 
//     double acc = max(20*c(5)*pow(t_crit,3) + 12*c(4)*square(t_crit) + 6*c(3)*t_crit + a0);
// 
//     if (acc < acc_max && acc > acc_max*.9)
//     {
//       break;
//     }
// 
// 
//     if (acc>acc_max)
//     {
//       lb = tf;
//       tf = (tf+ub)/2;
//     }
//     else
//     {
//       ub = tf;
//       tf = (tf+lb)/2;
//     }
//   }
// 
//   vec2 times;
//   times(0) = t0;
//   times(1) = tf+t0;
//   // 	cout << "numiters " << numiters << endl;
//   return Spline(c, times, 5);
// }
