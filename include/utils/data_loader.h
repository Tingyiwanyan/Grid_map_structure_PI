#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include "geometry_utils/Pose.h"
#include "functions.h"
#include "parameters.h"
#include "nc_data.h"
#include <gp.h>

#define DEBUG
#ifdef DEBUG
#define _cout(expr) std::cout<<expr
#else
#define _cout(expr)
#endif

using namespace std;
namespace gu = geometry_utils;


namespace utils {
  
  class DataLoader {
  public:

    typedef boost::shared_ptr<DataLoader> Ptr;
    typedef boost::shared_ptr<const DataLoader> ConstPtr;

    DataLoader(const utils::Parameters::Ptr&);
    virtual ~DataLoader();

    //accessors

    vector<double> getLatitudes(void) {
      return latitudes;
    }

    vector<double> getLongitudes(void) {
      return longitudes;
    }

    vector<gu::Vec2>& getVecField(void) {
      return vec_field;
    }

    nc_data_t& getNCdata(void) {
      return nc_data;
    }
    
    nc_data_t& getRawNCdata(void) {
      return raw_nc_data;
    }

    std::vector<libgp::GaussianProcess *>& 
    getuGPs(){
      return u_gps;
    }
 
    std::vector<libgp::GaussianProcess *>& 
    getvGPs(){
      return v_gps;
    }
 
    vector<gu::Vec2> getVecField(unsigned int frame) {
      unsigned int n = nc_data.n_rows * nc_data.n_cols;
      //unsigned int n = nc_data.frames[frame].cells.size();
      vec_field.resize(n);
      for (unsigned int i = 0; i < nc_data.n_rows ; i++) {
        for (unsigned int j = 0; j < nc_data.n_cols; j++) {
          vec_field[i * nc_data.n_cols + j] = nc_data.frames[frame].cells[i][j].ocean_vec;
        }
      }
      return vec_field;
    }


    void loadParams(void);

    void importVecFieldData(void);

    //import a matrix from file, the file should contain a dense matrix 
    static void importMatrix(ifstream&, vector<vector<double> >&);

    //import a vector from file
    static void importVec(ifstream&, vector<double>&);

    static void displayMatrix(const vector<vector<double> >&);

  private:

    utils::Parameters::Ptr pParams;

    // define ocean currents vector field
    vector<double> longitudes, latitudes;
    vector<gu::Vec2> vec_field; //vector field, each element corresponds to a current-vector of a location, if multiple frames, it store the first frame

    nc_data_t nc_data;
    nc_data_t raw_nc_data;
    
    std::string file_name;
    unsigned int num_of_files;
    double scaling_x;
    double scaling_y;

    //Gaussian process data, n frames from nc_data
    std::vector<libgp::GaussianProcess *> u_gps;
    std::vector<libgp::GaussianProcess *> v_gps;

    //Gaussian Process paramters
    unsigned int input_dim;
    std::string cov_func;
    std::vector<double> cov_params;

  };


}//namespace

#endif

