#ifndef Info_CORE_H
#define Info_CORE_H

#include <assert.h>
#include <iterator>
#include <algorithm>
#include <queue>
#include <fstream>

#include <gp.h>

#include "geometry_utils/Pose.h"
#include "geometry_utils/Transform2.h"
#include "utils/parameters.h"
#include "utils/functions.h"
#include "utils/nc_data.h"
#include "mdp_methods/mdp_grid2d.h"
#include "gp_model.h"
#include "solver_interface.h"

using namespace geometry_utils;

namespace info_planner {
  typedef std::vector<info_path> info_paths;
  
  class Info {
  public:

    typedef boost::shared_ptr<Info> Ptr;
    typedef boost::shared_ptr<const Info> ConstPtr;

  protected:

    utils::Parameters::Ptr pParams;

  public:

    Info(const utils::Parameters::Ptr& pParams, const std::vector<double> &cov_params, const size_t &layers, const size_t &grid_x_size, const size_t &grid_y_size, const utils::nc_data_t &nc_data, const info_path &starting_points, const std::set<gp_point> &observations, Solver_Interface *solver);
    virtual ~Info();

    void pred(const info_paths &paths);
    
    info_paths run(const utils::nc_data_t &nc_data, const size_t &frame_count, const size_t &stages, const size_t &z);
    
    virtual void loadParams(void);

    void setGoals(const std::queue<Transform2>& goals, const size_t &robot_id) {
      q_goals[robot_id] = goals;
    }

    std::queue<Transform2>& getGoals(const size_t &robot_id) {
      return q_goals[robot_id];
    }
    
    void updateVarianceMap(double scale_x, double scale_y);
    void updateVarianceMap();

    void setVarMapSize(const size_t &lat_size, const size_t &lon_size);
    void setMdpMapSize(const size_t &lat_size, const size_t &lon_size);

    const std::vector<double> &getVarMap();
    
    void drawPath(const info_path &path);
    
    void draw(const gp_point &current_point, const std::vector<std::pair<gp_point, std::pair<double, double>>> &BV, const size_t &total_samples, const size_t &frame_count, const size_t &skip);
    info_paths final_paths_draw;

  private:
    const utils::nc_data_t nc_data;

    std::vector<std::queue<Transform2>> q_goals;
    size_t goal_count;

    GP_Model *gp_model_var;
    std::vector<double> varMap;
    size_t mdp_lat_size;
    size_t mdp_lon_size;

    info_path recursive(const utils::nc_data_t &original_nc_data, const size_t &stages, size_t current_layer, const size_t &point_num, double &total_mi_value, double &total_time);

//    std::ofstream time_file;

    cv::Mat image;
    void drawBackGround(const size_t &frame_count);
    
    info_path tsp_tour(const info_path &final_way_points);
    
    const size_t layers;
    const size_t grid_x_size;
    const size_t grid_y_size;
    
    size_t input_dim;
    std::string cov_func;
    std::vector<double> cov_params;
    size_t max_bv_size;
    double tolerance_error;
    
    info_path starting_points;
    std::set<gp_point> observations;
    
    Solver_Interface *solver;
    

    void WriteLengthResult(const std::vector<std::pair<size_t, size_t>> &path, const size_t &stages);
    void WriteTotalMutualInformation(const double &mi_value, const size_t &stages);
    void WriteTimeResult(const double &time, const size_t &stages, const size_t &count);
    void WritePathTimeResult(const double &time, const size_t &stages, const size_t &count);

    info_paths multiPath(info_path final_way_points);
    info_paths incrementalMultiPath(const info_path &way_points, double &time);

    info_paths singlePath(info_path final_way_points);
  public:
    size_t num_robots;
    double lambda;
    size_t rounds;
  };

}


#endif

