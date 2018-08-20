/*!
 * \file qp_spline.h
 */

#ifndef __QP_SPLINE_H_
#define __QP_SPLINE_H_

#include <list>
#include <vector>
#include <armadillo>


class Spline
{
public:
  Spline(arma::vec knotts, arma::vec wpts, arma::vec fixed_d, arma::vec fixed_d_idxs, double order, double sample_dt);
  Spline(arma::vec coefficients, arma::vec knotts, double order, double sample_dt);

  arma::vec getKnotTimes();
  arma::vec getCoeffs();
  int getNumWpts();

  double evalAtTime(double t, int deriv=0);

  int getSegmentIdx(double t);

  arma::vec getDensePath();
//   Spline quinticConnector(arma::vec X0, double t0, double acc_max);
//   static bool projectPointOnSplines(arma::vec p, std::vector<Spline> splineVec, double &tout);

private:

  arma::vec knottimes;
  arma::vec waypoints;
  arma::vec fixed_deriv;
  arma::vec fixed_deriv_idxs;
  double N; // number of spline coefficients (order+1)
  int numWpts;

  // Coefficients defined as p_n, p_n-1,...,p0 for each knot time, stacked into a vector
  arma::vec coeffs;
  arma::vec densePath;
  arma::vec densePathTimes;

  void fitSplineUnc();
  void setDensePath(double sample_dt);

};


#endif
