#ifndef __WAYPOINTS_H_GUARD
#define __WAYPOINTS_H_GUARD

#include <deque>

#include "structures.h"

void	waypointsFlightLoop(FlightBoard &, GPS &, IMU &, Logger &, std::deque<coord> &);

/* External variables */
extern bool		exitProgram;
extern int		state;
extern int		userState;
extern size_t	wp_it;

#endif
