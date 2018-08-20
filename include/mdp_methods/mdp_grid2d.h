#ifndef MDP_GRID2D_H
#define MDP_GRID2D_H

#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "geometry_utils/Pose.h"

using namespace std;

namespace mdp_planner{

  typedef 
  struct MDP_Grid2D 
  {

    typedef boost::shared_ptr<MDP_Grid2D> Ptr;
    typedef boost::shared_ptr<const MDP_Grid2D> ConstPtr;

    uint n_cols, n_rows;
    double resolution;
    point2d_t origin;

    std::vector<point2d_t> 	cell_centers;	
   
    MDP_Grid2D():n_cols(0), n_rows(0), resolution(0){}
    MDP_Grid2D(uint _n_cols, uint _n_rows, double _resolution, const point2d_t& _origin)
    {
      initializeGrid2D(_n_cols, _n_rows, _resolution, _origin);
      createGrids();
    }
    virtual ~MDP_Grid2D(){}


    virtual void initializeGrid2D(uint _n_cols, uint _n_rows, double _resolution, const point2d_t& _origin)
    {
      n_cols = _n_cols;
      n_rows = _n_rows;
      resolution = _resolution;
      origin = _origin;
    }


    //create grid based on given origin. 0 is indexed from bottom-left origin
    virtual void createGrids(void)
    {
      if(n_cols == 0 || n_rows == 0)
      {
        cout<<"mdp_grid2d: incorrect initialization of grid!"<<endl;
        return;
      }

      double cell_width = resolution;
      double cell_center_offset = cell_width/2;

      cell_centers.clear();
      uint id = 0;
      for(uint i=0; i<n_rows; i++)
        for(uint j=0; j<n_cols; j++){
          point2d_t p;
          p.x() = origin.x() + j*resolution + cell_center_offset;
          p.y() = origin.y() + i*resolution + cell_center_offset;
          cell_centers.push_back(p);
       }
    }


    //create grid within a bbx, 0 is indexed from top-left corner
    virtual void createGrids(double _bx_pos, double _bx_neg,
                        double _by_pos, double _by_neg,
                        uint _x_divide, uint _y_divide)
    {
      double x_cell_width = (double)(_bx_pos - _bx_neg)/_x_divide;
      double y_cell_width = (double)(_by_pos - _by_neg)/_y_divide;

      double x_offset = x_cell_width/2;
      double y_offset = y_cell_width/2;

      uint id = 0;
      for(uint i=0; i<_y_divide; i++)
        for(uint j=0; j<_x_divide; j++){
          point2d_t p;
          p.x() = _bx_neg + x_offset + j*x_cell_width;
          p.y() = _by_pos - y_offset - i*y_cell_width;
          cell_centers.push_back(p);
       }

       //also update n_cols n_rows (no resultion, may be not square shape)
       n_cols = _x_divide;
       n_rows = _y_divide;
       origin.set(_bx_neg, _by_neg);
    }


    // given a point, and the cell w/h of grid, get corresponding state
    uint getID(const point2d_t& p, const point2d_t& _origin, double cell_width, double cell_height)
    {
      assert(cell_width != 0 && cell_height != 0);

      if((static_cast<uint>((p.y() - _origin.y())/cell_height) >= n_rows) ||
                (static_cast<uint>((p.x() - _origin.x())/cell_width) >= n_cols) ){
        cerr<<"Error!! mdp_grid2d::getID point is outside of grid map!"<<endl;
        exit(0);
      }

      return static_cast<uint>((p.y() - _origin.y())/cell_height)*n_cols +
                    static_cast<uint>((p.x() - _origin.x())/cell_width);
    }


    uint getID(const point2d_t& p)
    {
       return getID(p, origin, resolution, resolution);
    }

  } mdp_grid2d_t;

}

#endif

