#ifndef __WAYPOINTS_H_GUARD
#define __WAYPOINTS_H_GUARD

#include <deque>

#include "structures.h"

void	waypointsFlightLoop(FlightBoard &, GPS &, IMU &, Buzzer &, Logger &, std::deque<coord> &);

/* External variables */
extern size_t	wp_it;

#endif
