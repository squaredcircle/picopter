#ifndef __WAYPOINTS_H_GUARD
#define __WAYPOINTS_H_GUARD

#include <deque>

void	waypointsFlightLoop(FlightBoard &, GPS &, IMU &, Logger &);

/* External variables */
extern std::deque<Coord_rad>	waypoints_list;

extern bool		exitProgram;
extern int		state;
extern int		userState;
extern size_t	wp_it;

#endif
