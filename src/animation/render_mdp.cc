
#include "animation/render_mdp.h"
#include <chrono>
#include "viz_tool/gldraw.h"
#include <stdlib.h>
#include <time.h>

namespace gu = geometry_utils;
using namespace std::chrono;

MethodManager::Ptr pMgr;

vector<point2d_t> Grids; //fill with pNet->cell_centers

bool PauseFlag = true;
double originX = 0.0, originY = 0.0, radius = 11, angle = 0.314, circle_count = 0, iter_count = 0, prev_state = -1;


/*
// for quadrotor
extern GLfloat RotorSpinAngle;
extern GLfloat WheelSpinAngle;
GLfloat RotorSpinSpeed = 60;
 */


namespace render_mdp {


  // equivalent of constructor

  void init(void) {

    pMgr = MethodManager::Ptr(new MethodManager);

    if (pMgr->getpParams()->getParamNode()["method_manager"].as<string>().compare("bagfile") != 0)
      getGrids();

  }


  //get grids after pointers have been initialized

  void getGrids(void) {


    if (!pMgr) {
      cout << "!!pMgr pointer not initialized." << endl;
      assert(0);
    }

    if (!pMgr->getpGrid2d()) {
      cout << "!!grid2d env pointer not initialized." << endl;
      assert(0);
    }

    ::Grids = pMgr->getpGrid2d()->cell_centers;

  }

  void
  drawMDP_States(const MDP_Net::Ptr& _pNet) {

    vector<mdp_state_t*> _states = _pNet->mdp_states;
    vector<point2d_t> _grids = _pNet->cell_centers;

    for (uint i = 0; i < _states.size(); i++) {

      mdp_state_t* s = _states[i];
      //don't draw spawned starts and goals
      if (s->spawn_parent != NULL) {
        //cout<<"not drawing starts or goals"<<endl;
        continue;
      }

      //for goal state
      if (_states[i]->type == GOAL) {
        glPushMatrix();
        glTranslatef(_grids[s->id].x(), _grids[s->id].y(), 0.11);
        //glScalef(1, 1, 0.1);
        //glutSolidCube(pMgr->getBoundsXYZ().x()*2/pMgr->getNumCols());
        //glCallList(model_list_cone);
        drawTargetSymbol(0.8 * pMgr->getBoundsXYZ().x() / pMgr->getNumCols(), 20);
        glPopMatrix();
      }//for obstacles
      else if (_states[i]->type == OBSTACLE) {
        glColor3f(0.3, 0.3, 0.3);
        glPushMatrix();
        glTranslatef(_grids[s->id].x(), _grids[s->id].y(), 0.1);
        glScalef(1, 1, 0.1);
        glutSolidCube(pMgr->getBoundsXYZ().x()*2 / pMgr->getNumCols());
        glPopMatrix();
      }
      //all normal states
      if (pMgr->getpParams()->getParamNode()["visualization"]["show_states_pos"].as<bool>()) {
        if (_states[i]->color)
          glColor3f(0, 0, 0);
        else
          glColor3f(0, 0, 1);
        double height = 0;
        glPushMatrix();
        glTranslatef(_grids[s->id].x(), _grids[s->id].y(), height);
        glutSolidCube(0.1 * (pMgr->getBoundsXYZ().x()*2 / pMgr->getNumCols()));
        glPopMatrix();
      }

    }//i

  }


void
drawMDP_Policies(const MDP_Net::Ptr& _pNet, const RGB &custom_color){

    vector<mdp_state_t*> _states = _pNet->mdp_states;
    vector<point2d_t> _grids = _pNet->cell_centers;

    for (uint i = 0; i < _states.size(); i++) {

      mdp_state_t* s = _states[i];
      //don't draw spawned starts and goals
      if (s->spawn_parent != NULL) {
        //cout<<"not drawing starts or goals"<<endl;
        continue;
      }

    //draw policy arrows
    //double arrow_linewidth=8*10/pMgr->getNumRows();
    double arrow_length=1.5;
    double arrow_linewidth=2;
    double arrow_pos_z = 0.1;
    for(uint j=0; j<s->actions.size(); j++){
      glPushMatrix();
        glColor3f(custom_color.r / 255.0, custom_color.g / 255.0, custom_color.b / 255.0);
        glTranslatef(_grids[s->id].x(), _grids[s->id].y(), arrow_pos_z);
        double arrow_scale = pMgr->getpParams()->getParamNode()["visualization"]["visual_arrow_scale"].as<double>();
        glScalef(arrow_scale, arrow_scale, 1); //changing z scale affects color/light?
        //case NORTH:
        if (s->actions[NORTH]) draw2DArrow(M_PI / 2, arrow_length, arrow_linewidth);
        //case NE:
        if (s->actions[NE]) draw2DArrow(M_PI / 4, arrow_length, arrow_linewidth);
        //case EAST:
        if (s->actions[EAST]) draw2DArrow(0, arrow_length, arrow_linewidth);
        //case SE:
        if (s->actions[SE]) draw2DArrow(-M_PI / 4, arrow_length, arrow_linewidth);
        //case SOUTH:
        if (s->actions[SOUTH]) draw2DArrow(-M_PI / 2, arrow_length, arrow_linewidth);
        //case SW:
        if (s->actions[SW]) draw2DArrow(-3 * M_PI / 4, arrow_length, arrow_linewidth);
        //case WEST:
        if (s->actions[WEST]) draw2DArrow(M_PI, arrow_length, arrow_linewidth);
        //case NW:
        if (s->actions[NW]) draw2DArrow(3 * M_PI / 4, arrow_length, arrow_linewidth);
        glPopMatrix();
      } //for j

    }//end for i

  }

  void
  drawStaticVecField(const vector<Vec2>& _vf, const vector<point2d_t>& _grids) {

    for (uint i = 0; i < _vf.size(); i++) {

      if (std::isnan(_vf[i].x()) || std::isnan(_vf[i].y()))
        continue;

      //draw vec field arrows
      double arrow_linewidth = 1;
      double dist_to_center = (pMgr->getpDisturb()->getSwirlCenter() - Vec2(_grids[i].x(), _grids[i].y())).norm();
      glPushMatrix();
      glTranslatef(_grids[i].x(), _grids[i].y(), 0.01);
      double re_scale = pMgr->getpParams()->getParamNode()["visualization"]["visual_vf_scale"].as<double>();
      viz_tool::draw2DArrow2(re_scale * _vf[i].x(), re_scale * _vf[i].y(), arrow_linewidth);
      glPopMatrix();

    }//end for i

  }

  void
  drawStaticDisturbances(const vector<Vec2>& _vf, const vector<point2d_t>& _grids) {

    for (uint i = 0; i < _vf.size(); i++) {
      if (std::isnan(_vf[i].x()) || std::isnan(_vf[i].y()))
        continue;

      //draw vec field arrows
      double arrow_linewidth = 1;
      double dist_to_center = (pMgr->getpDisturb()->getSwirlCenter() - Vec2(_grids[i].x(), _grids[i].y())).norm();
      glPushMatrix();
      //glColor3f(0.f, 0.f, 1.0f);
      if (pMgr->getpParams()->getParamNode()["disturbance"]["pattern"].as<string>().compare("uniform") == 0)
        glColor3f(0.f, 0.f, 1.0f);
      else
        viz_tool::colormap(dist_to_center / 300.0); //used when center vectors are big
      glTranslatef(_grids[i].x(), _grids[i].y(), 0.01);
      double re_scale = pMgr->getpParams()->getParamNode()["visualization"]["visual_vf_scale"].as<double>();
      viz_tool::draw2DArrow2(re_scale * _vf[i].x(), re_scale * _vf[i].y(), arrow_linewidth);
      glPopMatrix();

    }//end for i

  }

  void
  drawDynamicDisturbances(const Disturbance::Ptr& dp, const vector<point2d_t>& _grids) {

    // vec field from functions
    dp->updateDynamicVecField();

    for (uint i = 0; i < dp->getVecField().size(); i++) {

      if (std::isnan(dp->getVecField()[i].x()) || std::isnan(dp->getVecField()[i].y()))
        continue;

      //draw vec field arrows
      double arrow_linewidth = 1;
      double dist_to_center = (dp->getSwirlCenter() - Vec2(_grids[i].x(), _grids[i].y())).norm();
      glPushMatrix();
      // set color
      if (pMgr->getpParams()->getParamNode()["disturbance"]["pattern"].as<string>().compare("uniform") == 0)
        glColor3f(0.f, 0.f, 1.0f);
      else
        viz_tool::colormap(dist_to_center / 300.0); //used when center vectors are big
      //colormap(1.0/dist_to_center);  //used when center vectors are small
      glTranslatef(_grids[i].x(), _grids[i].y(), 0.01);
      double re_scale = pMgr->getpParams()->getParamNode()["visualization"]["visual_vf_scale"].as<double>();
      viz_tool::draw2DArrow2(re_scale * dp->getVecField()[i].x(), re_scale * dp->getVecField()[i].y(), arrow_linewidth);
      glPopMatrix();

    }//end for i

  }

  void
  drawImportedDisturbances(const Disturbance::Ptr& dp, const vector<point2d_t>& _grids) {

    // vec field from NC data processing by GPR
    dp->updateDynamicVecFieldNC();

    double cur_time = utils::WallClock::getWallTime();
    cur_time /= pMgr->getpParams()->getParamNode()["imported_vf"]["rescale_nc_interval"].as<double>();

    if (cur_time >= pMgr->getpData()->getNCdata().frames.size()) {
      cerr << "Finished all frames! GP now is doing prediction instead of interpolation!!" << endl;
      //exit(0);
    }

    /*
    // draw original frames from raw nc data
    static vector<Vec2> vf;
    static double vf_timer;	//control vf reploting frequency
    static int frame_id;
    vf_timer += utils::WallClock::getdt();
    double frame_interval = 1.0;
    if(vf.empty() || vf_timer > frame_interval){
      vf = dp->getVecField(frame_id++);
      cout<<"frame id: "<<frame_id<<", frame time: "<<dp->getNCdata().frames[frame_id].time<<endl;
      vf_timer = 0;
      if(frame_id >= dp->getNCdata().frames.size())
        frame_id = 0;
    }
     */

    for (uint i = 0; i < dp->getVecField().size(); i++) {

      if (std::isnan(dp->getVecField()[i].x()) || std::isnan(dp->getVecField()[i].y()))
        continue;

      //draw vec field arrows
      double arrow_linewidth = 1;
      glPushMatrix();
      // set color
      glColor3f(0.f, 0.f, 1.0f);
      //colormap(dp->getVecField()[i].norm());  //used when center vectors are small
      glTranslatef(_grids[i].x(), _grids[i].y(), 0.01);
      double re_scale = pMgr->getpParams()->getParamNode()["visualization"]["visual_vf_scale"].as<double>();
      viz_tool::draw2DArrow2(re_scale * dp->getVecField()[i].x(), re_scale * dp->getVecField()[i].y(), arrow_linewidth);
      glPopMatrix();

    }//end for i

  }

  void
  drawImportedDisturbancesByFrame(const Disturbance::Ptr& dp, const vector<point2d_t>& _grids, const int &frame_id) {
    // draw original frames from raw nc data
    vector<Vec2> vf = dp->getVecField(frame_id);

    for (uint i = 0; i < vf.size(); i++) {

      if (std::isnan(vf[i].x()) || std::isnan(vf[i].y()))
        continue;

      //draw vec field arrows
      double arrow_linewidth = 1;
      glPushMatrix();
      // set color
      glColor3f(0.f, 0.f, 1.0f);
      //colormap(dp->getVecField()[i].norm());  //used when center vectors are small
      glTranslatef(_grids[i].x(), _grids[i].y(), 0.01);
      double re_scale = pMgr->getpParams()->getParamNode()["visualization"]["visual_vf_scale"].as<double>();
      viz_tool::draw2DArrow2(re_scale * vf[i].x(), re_scale * vf[i].y(), arrow_linewidth);
      glPopMatrix();

    }//end for i

  }

  void
  drawInfoMap(const MDP_Net::Ptr& _pNet, const Info::Ptr& info, const vector<point2d_t>& _grids) {

    assert(_pNet->cell_centers.size() == _grids.size());
    //cout<<"Grids: "<<_grids.size()<<" info: "<<info->getVarMap(0).size()<<endl;
    for (uint i = 0; i < _pNet->mdp_states.size(); i++) {

      mdp_state_t* s = _pNet->mdp_states[i];
      //don't draw spawned starts and goals
      if (s->spawn_parent != NULL || s->type == OBSTACLE) {
        //cout<<"not drawing starts, goals, obstacles"<<endl;
        continue;
      }

      //draw variance colormap
      double max_val = 500;
      double offset = -0.5;
      glPushMatrix();
      glTranslatef(0, 0, offset);
      glBegin(GL_QUADS);
      viz_tool::colormap(info->getVarMap()[s->id] / max_val);
      glVertex3f(_grids[s->id].x(), _grids[s->id].y(), 0);
      viz_tool::colormap(info->getVarMap()[s->successors[NORTH]->id] / max_val);
      glVertex3f(_grids[s->successors[NORTH]->id].x(), _grids[s->successors[NORTH]->id].y(), 0);
      viz_tool::colormap(info->getVarMap()[s->successors[NE]->id] / max_val);
      glVertex3f(_grids[s->successors[NE]->id].x(), _grids[s->successors[NE]->id].y(), 0);
      viz_tool::colormap(info->getVarMap()[s->successors[EAST]->id] / max_val);
      glVertex3f(_grids[s->successors[EAST]->id].x(), _grids[s->successors[EAST]->id].y(), 0);
      glEnd();
      glPopMatrix();

    }//end for i

  }

  void drawAgent(const gu::Transform2& tf2) {

    //glScalef(1.5,1.5,1.5); // a larger value would make the obj  black!?

    //test origin position
    //glutSolidCube(2);

    //using glmDraw makes rendering slow
    //glmDraw(model, GLM_SMOOTH | GLM_MATERIAL);
    glPushMatrix();
    glTranslatef(tf2.translation.x(), tf2.translation.y(), 0.12);
    //glutPrint(0, 0, glutFonts[4], text, 0.f, 0.f, 0.f, 1.0f);
    glPushMatrix();
    gu::Vec3 u(0, 1, 0); //glider default orientation
    //gu::Vec3 v(orient.x(), orient.y(), 0); //3d actual orientation
    gu::Vec3 v(std::cos(tf2.rotation.angle()), std::sin(tf2.rotation.angle()), 0); //3d actual orientation
    /* works only for 2D
    double angle = atan2( (u.x()*v.y() - u.y()*v.x()), u.dot(v) )*180/M_PI;
    glRotatef(angle, 0, 0, 1);
     */
    gu::Quat q = getQuaternion(u, v);
    gu::Quat axis_angle = q.axisAngle();
    //viz_tool::draw2DArrow2(5*v.x(), 5*v.y(),  1);
    glRotatef(axis_angle.w()*180 / M_PI, axis_angle.x(), axis_angle.y(), axis_angle.z());
    glPushMatrix();
    glRotatef(90, 1,0,0);
    glScalef(0.4, 0.4, 0.4);
    drawMobileRobot();
    glPopMatrix();
    //drawQuadrotor(true);
    //glScalef(0.002, 0.002, 0.002);
    glPopMatrix();
    glPopMatrix();

  }

  void
  drawTrajectory(const vector<geometry_utils::Transform2>& trajectory) {

    // draw trajectory, note: it can be expensive if the resolution is high
    if (pMgr->getpParams()->getParamNode()["visualization"]["show_trajectory"].as<bool>()) {
      glEnable(GL_COLOR_MATERIAL);
      glColor3f(0.0f, 1.0f, 0.0f);
      //paths for internal nodes
      if (!trajectory.empty()) {
        for (uint j = 0; j < trajectory.size() - 1; j++) {
          draw2DThickLine(trajectory[j].translation, trajectory[j + 1].translation, 0.3);
        }
      }
    }

  }

  void
  drawDynamicObstacle() {

      int state;
      double x, y;
      mdp_planner::MDP_Net::Ptr pNet;

      x = originX + cos(angle*circle_count)*radius;
      y = originY + sin(angle*circle_count)*radius;

      point2d_t new_pt;
      new_pt.x() = x;
      new_pt.y() = y;
      state = pMgr->getpGrid2d()->getID(new_pt);

      if(iter_count == 0){

          cout << "State New: " << state << endl;
          pNet = pMgr->getpNet();
          pNet->reachablestates.clear();

          if(prev_state != -1){
              cout << "State Previous: " << prev_state << endl;
              pNet->setNonObstacleStateValues(prev_state, prev_state+2);
          }

          pNet->setObstacleStateValues(state, state+2, -100);

          pMgr->getpSSP()->initTransMatrix();
          double fpt_val_n = pMgr->getpSSP()->meanFirstPassageTime(pNet->getState(pMgr->tf2_starts[0].translation), pNet->getState(pMgr->tf2_goals[0].translation)->id);

          //pMgr->getpMDP()->iterations();
          //pMgr->getpMDP()->optimalActionTransitionDistribution(pNet->mdp_states);

          std::cout << "Unreachable states to High Goal: " << pNet->reachablestates.size() << std::endl;

          if(true)
          {
              std::cout << "Need to update policy locally" << std::endl;
              uint num_iterations = pMgr->getpParams()->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
              for (uint i = 0; i < num_iterations; i++)
              {
                  cout << i << " " << std::flush;
                  pMgr->getpMDP()->valueIterationLocalStates(pNet->mdp_states,pNet->reachablestates,"forward");
              }
              //pMDP->optimalActionTransitionDistribution(pNet->mdp_states);

          }
          else{
              std::cout << "No need to update policy" << std::endl;
          }

      }

      ++iter_count;
      if(iter_count == 50) {

        if(((int)circle_count) % 4 == 0)
        {
            pMgr->getpMDP()->iterations();
            pMgr->getpMDP()->optimalActionTransitionDistribution(pMgr->getpNet()->mdp_states);
        }
        circle_count = (circle_count == 20) ? 0 : circle_count + 0.02 * iter_count;
        iter_count = 0;
        prev_state = state;

      }

      glColor3f(0.3, 0.3, 0.3);
      glPushMatrix();
      glTranslatef(pMgr->getpNet()->cell_centers[state].x(), pMgr->getpNet()->cell_centers[state].y(), 0.1);
      //glTranslatef(x, y, 0.1);
      glScalef(1, 1, 0.1);
      glutSolidCube(pMgr->getBoundsXYZ().x()*2 / pMgr->getNumCols());
      glPopMatrix();

      glColor3f(0.3, 0.3, 0.3);
      glPushMatrix();
      glTranslatef(pMgr->getpNet()->cell_centers[state+1].x(), pMgr->getpNet()->cell_centers[state+1].y(), 0.1);
      //glTranslatef(x+1, y, 0.1);
      glScalef(1, 1, 0.1);
      glutSolidCube(pMgr->getBoundsXYZ().x()*2 / pMgr->getNumCols());
      glPopMatrix();

      glColor3f(0.3, 0.3, 0.3);
      glPushMatrix();
      glTranslatef(pMgr->getpNet()->cell_centers[state+2].x(), pMgr->getpNet()->cell_centers[state+1].y(), 0.1);
      //glTranslatef(x+2, y, 0.1);
      glScalef(1, 1, 0.1);
      glutSolidCube(pMgr->getBoundsXYZ().x()*2 / pMgr->getNumCols());
      glPopMatrix();

  }

  void
  drawPlannedWaypoints(const gu::Transform2& cur_tf2, const deque<gu::Transform2>& waypoints) {

    // draw to-go waypoints
    if (pMgr->getpParams()->getParamNode()["visualization"]["show_way_points"].as<bool>()) {
      glEnable(GL_COLOR_MATERIAL);
      if (!waypoints.empty()) {
        //draw way points
        for (uint j = 0; j < waypoints.size(); j++) {
          glPushMatrix();
          glColor3f(0, 0, 1);
          glTranslatef(waypoints[j].translation.x(), waypoints[j].translation.y(), 0);
          glutSolidSphere(0.1 * (pMgr->getBoundsXYZ().x()*2 / pMgr->getNumCols()), 5, 5);
          glPopMatrix();
        }
        //draw connected path
        glBegin(GL_LINE_STRIP);
        glLineWidth(2);
        glColor3f(0.0f, 0.4f, 0.0f);
        glVertex3f(cur_tf2.translation.x(), cur_tf2.translation.y(), 0.02);
        for (uint j = 0; j < waypoints.size(); j++)
          glVertex3f(waypoints[j].translation.x(), waypoints[j].translation.y(), 0.02);
        glEnd();
      }
    }
  }

  void
  drawMDP_All(void) {
    //std::cout<<"Im here at drawall"<<std::endl;

    //to set color, need to turn on below, and turn off texture if it is on
    glEnable(GL_COLOR_MATERIAL);
    glClear(GL_COLOR_BUFFER_BIT);

    //draw grids
    if (pMgr->getpParams()->getParamNode()["visualization"]["show_grids"].as<bool>())
      drawGridEnv(-pMgr->getBoundsXYZ().x(), pMgr->getBoundsXYZ().x(),
            -pMgr->getBoundsXYZ().y(), pMgr->getBoundsXYZ().y(),
            pMgr->getNumRows(), pMgr->getNumCols());

    //mdp stuff

    if (pMgr->getpParams()->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
      drawMDP_States(pMgr->getpNet());
      vector<point2d_t> _grids = pMgr->getpNet()->cell_centers;
      /*
      for (int i = 0; i < pMgr->getpNet()->mdp_states.size(); i++) {

        mdp_state_t* s = pMgr->getpNet()->mdp_states[i];
        //don't draw spawned starts and goals
        if (s->spawn_parent != NULL || s->type == OBSTACLE) {
          //cout<<"not drawing starts, goals, obstacles"<<endl;
          continue;
        }

        //draw variance colormap
        double max_val = 500;
        double offset = -0.5;
        glPushMatrix();
        glTranslatef(0, 0, offset);
        glBegin(GL_QUADS);
        viz_tool::colormap(pMgr->getpNet()->states_fpt_final[s->id] / max_val);
        glVertex3f(_grids[s->id].x(), _grids[s->id].y(), 0);
        glEnd();
        glPopMatrix();
      }
      */

      if (pMgr->getpParams()->getParamNode()["visualization"]["show_policy"].as<bool>())
        drawMDP_Policies(pMgr->getpNet(), pMgr->getpNet()->getPolicyColor());
    } else {
      for (unsigned int i = 0; i < pMgr->getpRobots().size(); i++) {
        drawMDP_States(pMgr->getpRobots()[i]->getpControl()->getpNet());
        if (pMgr->getpParams()->getParamNode()["visualization"]["show_policy"].as<bool>()) {
          drawMDP_Policies(pMgr->getpRobots()[0]->getpControl()->getpNet(), pMgr->getpRobots()[0]->getpControl()->getpNet()->getPolicyColor()); // one set of policy
        }
      }
    }

    //dynamic obstacles
    //drawDynamicObstacle();

    //disturbance vector field
    /*
    if (pMgr->getpParams()->getParamNode()["visualization"]["show_vec_field"].as<bool>()) {
      string method = pMgr->getpParams()->getParamNode()["method_manager"].as<string>();
      if (method.compare("imported_vf") == 0) {
        //NC data as input
        drawImportedDisturbances(pMgr->getpDisturb(), ::Grids);
      } else if (method.compare("info_plan") == 0) {
        drawImportedDisturbances(pMgr->getpDisturb(), ::Grids);
      } else {
        if (pMgr->getpParams()->getParamNode()["disturbance"]["translating_center"].as<bool>() ||
                pMgr->getpParams()->getParamNode()["disturbance"]["rotating_vectors"].as<bool>())
          drawDynamicDisturbances(pMgr->getpDisturb(), ::Grids);
        else
          drawStaticDisturbances(pMgr->getpDisturb()->getVecField(), ::Grids);
      }
    }
    */
    //draw info map

    if (pMgr->getpInfo()) {
    //if (5<10){
      drawInfoMap(pMgr->getpNet(), pMgr->getpInfo(), ::Grids);
    }

    // draw topo, all connection lines
    if (pMgr->getpParams()->getParamNode()["visualization"]["show_stoch_topo"].as<bool>()) {
      glColor3f(0.0f, 0.6f, 1.0f);
      //glColor3f(155.0/255, 205.0/255, 155.0/255);
      glLineWidth(1);
      glBegin(GL_LINES);
      // the internal nodes
      for (uint i = 0; i < pMgr->getpNet()->mdp_states.size(); i++)
        for (vector<mdp_state_t*>::iterator itr = pMgr->getpNet()->mdp_states[i]->successors.begin(); itr != pMgr->getpNet()->mdp_states[i]->successors.end(); itr++) {
          glVertex3f(::Grids[pMgr->getpNet()->mdp_states[i]->id].x(), ::Grids[pMgr->getpNet()->mdp_states[i]->id].y(), 1e-3);
          glVertex3f(::Grids[(*itr)->id].x(), ::Grids[(*itr)->id].y(), 1e-3);
        }
      glEnd();
    }

    //draw robot and trajectory, etc

    for (uint i = 0; i < pMgr->getpRobots().size(); i++) {
      //std::cout<<"Im here in draw robot"<<std::endl;
      AUVmodel::Ptr r = pMgr->getpRobots()[i];
      AUVcontroller::Ptr pc = r->getpControl();
      //draw to-go waypoints
      drawPlannedWaypoints(pc->cur_tf2, pc->waypoints);
      //draw trajectory
      drawTrajectory(pc->trajectory);
      //draw agent
      //srand(time(NULL));
      //int isecret = rand() %10 + 1;
      //viz_tool::draw2DArrow(4+isecret,4,5);
      drawAgent(r->gettf2());
    }

    //show simulation time
    char text[16];
    sprintf(text, "%4f", utils::WallClock::getWallTime());
    glPushMatrix();
    glTranslatef(-28, 28, 0.2);
    glutPrint(0, 0, glutFonts[4], text, 0.f, 0.7f, 0.f, 1.0f);
    glPopMatrix();

  }


  //wrapper for wall drawing

  void
  drawWallsWrapper(void) {

    GLuint texture2[2][3]; // storage for textures, each 3 modes
    drawHorizonWalls(texture2);

  }


  //define floor vertices used in findPlane & drawReflection & drawfloor
  GLfloat floorPlane[4];
  GLfloat f_w = 50; //floor width
  GLfloat floorVertices[4][3] = {
    { -f_w, f_w, 0.0},
    { f_w, f_w, 0.0},
    { f_w, -f_w, 0.0},
    { -f_w, -f_w, 0.0},
  };


  //wrapper for find plane func

  void
  findPlaneWrapper(void) {

    /* Setup floor plane for projected shadow calculations. */
    findPlane(floorPlane, floorVertices[1], floorVertices[2], floorVertices[3]);

  }

  void
  render(void) {

    if (pMgr->getpParams()->getParamNode()["method_manager"].as<string>().compare("bagfile") == 0) {
      render_bagfile();
    } else
      render_realtime();

  }

  void
  render_realtime(void) {
    if (PauseFlag)
      utils::WallClock::pauseWallClock();
    else
      utils::WallClock::resumeWallClock();

//    static Transform2 dynamic_start_tf2, dynamic_goal_tf2;
    for (uint i = 0; i < pMgr->getpRobots().size(); i++) {
      //std::cout<<"IM here in realtime"<<std::endl;
      AUVmodel::Ptr r = pMgr->getpRobots()[i];
      // Info Plan
      if (pMgr->getpParams()->getParamNode()["method_manager"].as<string>().compare("info_plan") == 0 &&
              pMgr->getpParams()->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0) {
          std::cout<<"Im here at info_plan"<<std::endl;
        mdp_planner::MDP_Net::Ptr pNet;
        if (!pMgr->getpParams()->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
          pNet = r->getpControl()->getpNet();
        } else {
          pNet = pMgr->getpNet();
        }
        Transform2 tf2 = r->gettf2();

        if (pNet->getState(tf2.translation)->type == GOAL) {
          if (pMgr->getpInfo()->getGoals(i).empty()) {
            cout << "robot " << i << " finished all goals!" << endl;
            continue;
          }
          std::cout << "robot " << i << " reached the goal " << tf2.translation << std::endl;

          // gggggggggggggggggggg
          vector<double> longitudes = pMgr->getpData()->getLongitudes(),
                  latitudes = pMgr->getpData()->getLatitudes();
          double x_min = *std::min_element(longitudes.begin(), longitudes.end());
          double x_max = *std::max_element(longitudes.begin(), longitudes.end());
          double y_min = *std::min_element(latitudes.begin(), latitudes.end());
          double y_max = *std::max_element(latitudes.begin(), latitudes.end());

          double x_length = x_max - x_min;
          double y_length = y_max - y_min;
          point2d_t origin(-pMgr->getBoundsXYZ().x(), -pMgr->getBoundsXYZ().y());
          Vec2 goal = tf2.translation - origin;
          double ratio_x = 2 * pMgr->getBoundsXYZ().x() / x_length;
          double ratio_y = 2 * pMgr->getBoundsXYZ().y() / y_length;
          double scale_x = goal.x() / (ratio_x * x_length);
          double scale_y = goal.y() / (ratio_y * y_length);
          pMgr->getpInfo()->updateVarianceMap(scale_x, scale_y);
          // gggggggggggggggggggg

          pMgr->getpInfo()->getGoals(i).pop();

          // Reset types
          pNet->getState(tf2.translation)->type = START;
          pNet->getState(pMgr->getpInfo()->getGoals(i).front().translation)->type = GOAL;

          if (!pMgr->getpParams()->getParamNode()["mdp_methods"]["shared_policy"].as<bool>()) {
            vector<Transform2> tf2_goals;
            tf2_goals.push_back(pMgr->getpInfo()->getGoals(i).front());
            r->getpControl()->MDPIterations(tf2_goals);
          } else {
//            pMgr->getpMDP()->cleanStates();
            pMgr->getpMDP()->iterations();
          }

          utils::WallClock::resetDt();
          //r->getpControl()->getClock().resetClock();
        }
        // SSP
      } else if (pMgr->getpParams()->getParamNode()["method_manager"].as<string>().compare("expected_ssp") == 0 &&
              pMgr->getpParams()->getParamNode()["macro_controller"].as<string>().compare("mdp_policy") == 0) {
        // TODO:
      }

      // evolve vehicle pose
      if (!PauseFlag) {
        r->poseEvolveInEnvironment();
//        if (pMgr->getpInfo()) {
//          pMgr->getpInfo()->updateVarianceMap();
//        }
      }
    }//for

    //draw the real objects
    drawMDP_All();

    /* fancy rendering with shadow, reflection etc */
    //just draw the reflection
    //drawReflection(light_position, floorVertices, drawAgents);

    // if reflection is not used, the floor should be drawn separately. see last part of drawReflection
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.9, 1.0, 1.0, 0.5);
    drawFloor(floorVertices, 16);
    glDisable(GL_BLEND);

#if 0
    //just draw the shadow, ugly though
    //GLfloat floorShadow[4][4];
    //drawShadow(floorShadow, drawAgents);

    //just to get the shadow by using glCallList, w/o it, no shawdow!!
    glPushMatrix();
    glScalef(0.01, 0.01, 0.01);
    glCallList(model_list_robot); // see where origin point is
    glPopMatrix();

    //draw the walls
    //drawWallsWrapper();

    // the rotor rotating angle updates
    //RotorSpinAngle+=RotorSpinSpeed;
#endif

  }

  void
  render_bagfile(void) {
  }

  void makechange(void){
      utils::WallClock::pauseWallClock();

      mdp_planner::MDP_Net::Ptr pNet;
      pNet = pMgr->getpNet();
      uint index41 = pMgr->getpParams()->getParamNode()["obstacles"]["bbx24a"].as<int>();
      uint index42 = pMgr->getpParams()->getParamNode()["obstacles"]["bbx24b"].as<int>();

      pNet->reachablestates.clear();
      pNet->setObstacleStateValues(index41, index42, -100);


      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      pMgr->getpSSP()->initTransMatrix();
      double fpt_val_n = pMgr->getpSSP()->meanFirstPassageTime(pNet->getState(pMgr->tf2_starts[0].translation), pNet->getState(pMgr->tf2_goals[0].translation)->id);

      //pMgr->getpMDP()->iterations();
      //pMgr->getpMDP()->optimalActionTransitionDistribution(pNet->mdp_states);

      std::cout << "Unreachable states to High Goal: " << pNet->reachablestates.size() << std::endl;

      if(true)
      {
          std::cout << "Need to update policy locally" << std::endl;

          uint num_iterations = pMgr->getpParams()->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
          for (uint i = 0; i < num_iterations; i++)
          {
              cout << i << " " << std::flush;
              pMgr->getpMDP()->valueIterationLocalStates(pNet->mdp_states,pNet->reachablestates,"forward");
          }
          //pMDP->optimalActionTransitionDistribution(pNet->mdp_states);

      }
      else{
          std::cout << "No need to update policy" << std::endl;
      }
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>( t2 - t1 ).count();
      cout << "\n\nTime Taken FPT: " << duration/1000000.0 << " seconds\n" << endl;

      utils::WallClock::resumeWallClock();

  }


  void makechange2(void){
      utils::WallClock::pauseWallClock();

      mdp_planner::MDP_Net::Ptr pNet;
      pNet = pMgr->getpNet();
      uint index41 = pMgr->getpParams()->getParamNode()["obstacles"]["bbx25a"].as<int>();
      uint index42 = pMgr->getpParams()->getParamNode()["obstacles"]["bbx25b"].as<int>();

      pNet->reachablestates.clear();
      pNet->setObstacleStateValues(index41, index42, -100);


      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      pMgr->getpSSP()->initTransMatrix();
      double fpt_val_n = pMgr->getpSSP()->meanFirstPassageTime(pNet->getState(pMgr->tf2_starts[0].translation), pNet->getState(pMgr->tf2_goals[0].translation)->id);

      //pMgr->getpMDP()->iterations();
      //pMgr->getpMDP()->optimalActionTransitionDistribution(pNet->mdp_states);

      std::cout << "Unreachable states to High Goal: " << pNet->reachablestates.size() << std::endl;

      if(true)
      {
          std::cout << "Need to update policy locally" << std::endl;

          uint num_iterations = pMgr->getpParams()->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
          for (uint i = 0; i < num_iterations; i++)
          {
              cout << i << " " << std::flush;
              pMgr->getpMDP()->valueIterationLocalStates(pNet->mdp_states,pNet->reachablestates,"forward");
          }
          //pMDP->optimalActionTransitionDistribution(pNet->mdp_states);

      }
      else{
          std::cout << "No need to update policy" << std::endl;
      }
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>( t2 - t1 ).count();
      cout << "\n\nTime Taken FPT: " << duration/1000000.0 << " seconds\n" << endl;

      utils::WallClock::resumeWallClock();

  }

  void makechange3(void){
      utils::WallClock::pauseWallClock();

      mdp_planner::MDP_Net::Ptr pNet;
      pNet = pMgr->getpNet();
      uint index43 = pMgr->getpParams()->getParamNode()["obstacles"]["bbx26a"].as<int>();
      uint index44 = pMgr->getpParams()->getParamNode()["obstacles"]["bbx26b"].as<int>();

      pNet->reachablestates.clear();
      pNet->setObstacleStateValues(index43, index44, -100);


      high_resolution_clock::time_point t1 = high_resolution_clock::now();
      pMgr->getpSSP()->initTransMatrix();
      double fpt_val_n = pMgr->getpSSP()->meanFirstPassageTime(pNet->getState(pMgr->tf2_starts[0].translation), pNet->getState(pMgr->tf2_goals[0].translation)->id);

      //pMgr->getpMDP()->iterations();
      //pMgr->getpMDP()->optimalActionTransitionDistribution(pNet->mdp_states);

      std::cout << "Unreachable states to High Goal: " << pNet->reachablestates.size() << std::endl;

      if(true)
      {
          std::cout << "Need to update policy locally" << std::endl;

          uint num_iterations = pMgr->getpParams()->getParamNode()["mdp_methods"]["num_iterations"].as<unsigned int>();
          for (uint i = 0; i < num_iterations; i++)
          {
              cout << i << " " << std::flush;
              pMgr->getpMDP()->valueIterationLocalStates(pNet->mdp_states,pNet->reachablestates,"forward");
          }
          //pMDP->optimalActionTransitionDistribution(pNet->mdp_states);

      }
      else{
          std::cout << "No need to update policy" << std::endl;
      }
      high_resolution_clock::time_point t2 = high_resolution_clock::now();
      auto duration = duration_cast<microseconds>( t2 - t1 ).count();
      cout << "\n\nTime Taken FPT: " << duration/1000000.0 << " seconds\n" << endl;

      utils::WallClock::resumeWallClock();

  }


  void
  key(unsigned char key, int x, int y) {

    switch (key) {
      case 'p':
      case 'P':
        PauseFlag = !PauseFlag;
        //drawMDP_All();
        break;
      case ESCAPE:
      case 'q':
      case 'Q':
        //drawMDP_All();
        //do some cleanup
        break;
      case 'c':
      case 'C':
        makechange();
        //do some cleanup
        break;
      case 'd':
      case 'D':
        makechange2();
        //do some cleanup
        break;
      case 'f':
      case 'F':
        makechange3();
        //do some cleanup
        break;
    }

  }

  void
  idle(void) {

    render();

  }


} //end namespace
