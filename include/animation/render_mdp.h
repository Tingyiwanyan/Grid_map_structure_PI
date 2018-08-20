
#ifndef RENDER_MDP_H
#define RENDER_MDP_H

#include "viz_tool/canvas.h"
#include "viz_tool/shadow.h"
#include "viz_tool/floor.h"
#include "viz_tool/tex_walls.h"
#include "viz_tool/gldraw.h"
#include "viz_tool/glfunc.h"

#include "method_manager/method_manager.h"

// ascii codes for various special keys 
#define ESCAPE 27

using namespace viz_tool;
namespace gu = geometry_utils;

namespace render_mdp
{

  void init(void);	//constructor equivalent

  void getGrids(void);

  void drawMDP_States(const MDP_Grid2D::Ptr&);
  void drawMDP_Policies(const MDP_Grid2D::Ptr&, const RGB &custom_color);

  void drawStaticVecField(const vector<Vec2>&, const vector<point2d_t>&);
  void drawStaticDisturbances(const vector<Vec2>&, const vector<point2d_t>&);
  void drawDynamicDisturbances(const Disturbance::Ptr&, const vector<point2d_t>&);
  void drawImportedDisturbances(const Disturbance::Ptr&, const vector<point2d_t>&);
  void drawImportedDisturbancesByFrame(const Disturbance::Ptr&, const vector<point2d_t>&);

  void drawAgent(const gu::Transform2& tf2);
  void drawTrajectory(const vector<gu::Transform2>&);
  void drawPlannedWaypoints(const gu::Transform2& cur_tf2, const deque<gu::Transform2>&);

  //put together
  void drawMDP_All(void);

  //for floor rendering
  void findPlaneWrapper(void);

  //not used for mdp
  void drawWalls(void);

  //rendering based on realtime simulation
  void render_realtime(void);

  //rendering based on bagfile input
  void render_bagfile(void);

  //render wrapper
  void render(void);

  /* gl routine */
  void key(unsigned char key, int x, int y);
  void idle(void);

};


#endif

