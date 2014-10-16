#ifndef __WAYPOINTS_LOOP5_H_INCLUDED__
#define __WAYPOINTS_LOOP5_H_INCLUDED__


#include "hardware.h"
#include "logger.h"
#include <deque>
#include "navigation_structures.h"
#include <string>

void	waypoints_loop5(hardware &, Logger &, std::deque<coord> &, std::string = "");

/* External variables */
extern size_t	wp_it;
extern bool 	repeatLoop;

#endif// __WAYPOINTS_LOOP4_5_INCLUDED__
