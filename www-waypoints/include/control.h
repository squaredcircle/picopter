#ifndef __CONTROL_H_GUARD
#define __CONTROL_H_GUARD

#include "structures.h"

/* Forward declarations */
double		calculate_distance(coord, coord);
double		calculate_bearing(coord, coord);
int			sign(float, float);
coord		getCoord(GPS*);
double		getYaw(IMU*);
bool		checkInPerth(coord*);
void		printFB_Data(FB_Data*);

void		setCourse(FB_Data*, double, double, double);
double		inferBearing(FlightBoard *fb, GPS *gps, Logger *logs);

#endif
