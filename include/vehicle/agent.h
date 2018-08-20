#ifndef AGENT_H
#define AGENT_H

#include "geometry_utils/Transform2.h"
#include <deque>
#include <set>


typedef unsigned int AgtID;

typedef
struct Agent {

  Agent(){} 
  ~Agent(){}

  std::string	name;
  AgtID 	id;

  //char 	type;		//coloring

  //current/last tf2, current/last reference tf2
  geometry_utils::Transform2 cur_tf2, last_tf2, cur_ref_tf2, last_ref_tf2;

  std::set<AgtID> neighbors;

  std::deque<geometry_utils::Transform2> waypoints;		//to-go way points	

  std::vector<geometry_utils::Transform2> trajectory;		//actual trace of motion

} agent_t;


#endif

