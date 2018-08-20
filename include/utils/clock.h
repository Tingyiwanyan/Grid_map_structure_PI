
#ifndef _UTILS_CLOCK_H
#define _UTILS_CLOCK_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <assert.h>
#include <algorithm>    // std::max
#include <iostream>

namespace utils
{

// a global wall clock used,  e.g., to tick simulation time
class WallClock
{

public:

  WallClock(void){}
  ~WallClock(void){}

  static void resetStartTime(void){ t = 0; };
  static void pauseWallClock(void){ pause = true; }
  static void resumeWallClock(void){ pause = false; }

  static double getdt(void){ return dt; }

  static void resetDt(void);

  static double getRealTime(void);
 
  static double getWallTime(void){ return t; }

  static double getLastWallTime(void) { return t_last; }

  static bool dtIsSet(void) { return dt_is_set; }
  
  static void resetWallClock(void);

  // evolve time for t, dt
  static void runWallClock(void);


private:

  static double t, t_last, dt;	//elapsed time, last time stamp, dt
  static bool pause; 		//pause wall clock
  static bool dt_is_set;

};


///////////////////////////////////////////////////////////////////////


// a local clock used by individual vehicle, i.e., a stop watch
class Clock
{

public:

  Clock(void):t(0), t_last(0), pause(false){}
  ~Clock(void){}

  void resetStartTime(void){ t = 0; };
  void pauseClock(void){ pause = true; t_last=WallClock::getRealTime(); }
  void resumeClock(void){ pause = false; }

  //dt is used from global wall clock since it runs all the time
  double getdt(void){ return WallClock::getdt(); }

  double getTime(void){ return t; }

  double getLastTime(void) { return t_last; }

  void resetClock(void);

  // evolve time for t, dt
  void runClock(void);

private:

  double t, t_last;
  bool pause; 	

};


}


#endif

