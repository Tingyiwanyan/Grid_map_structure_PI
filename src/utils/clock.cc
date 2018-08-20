
#include "utils/clock.h"

namespace utils
{

double WallClock::t=0, WallClock::t_last=0, WallClock::dt=0;	//wall clock
bool WallClock::pause = false;
bool WallClock::dt_is_set = false;


double
WallClock::getRealTime(void){

  struct timeval curr;
  gettimeofday(&curr,NULL);
  return curr.tv_sec + curr.tv_usec / 1e6;

}


void
WallClock::resetDt(void){

  dt = 0; 
  t_last = getRealTime(); 
	//reason: frames got stuck at expensive computation, dt between two successive frames can be very large. here force it to omit long lost last frame

}


void
WallClock::resetWallClock(void){

  resetDt();
  t = 0; 
  dt_is_set = true;

}


void
WallClock::runWallClock(void){

  if(!dt_is_set){
    //re-init t_last
    t_last = getRealTime();
    dt_is_set = true;
    return;
  }

  dt = getRealTime() - t_last;
  t_last = getRealTime();
  if(!pause){
    t += std::max(dt, 0.);
  }

}


//////////////////////////////////////////////////////////////////

void
Clock::resetClock(void){

  t = 0; 

}


void
Clock::runClock(void){

  if(!WallClock::dtIsSet()){
    //re-init t_last
    t_last = WallClock::getRealTime();
    return;
  }

  double dt = WallClock::getdt();
  t_last = WallClock::getRealTime();
  if(!pause){
    t += std::max(dt, 0.);
  }

}


}


