#ifndef __CONTROL_H_GUARD
#define __CONTROL_H_GUARD

/* Forward declarations */
double		calculate_distance(Coord_rad, Coord_rad);
double		calculate_bearing(Coord_rad, Coord_rad);
int			sign(float, float);
Coord_rad	getCoordDeg(GPS*);
double		getYaw(IMU*);
bool		checkInPerth(Coord_rad*);
void		printFB_Data(FB_Data*);

void		setCourse(FB_Data*, double, double, double);
bool		initialise(FlightBoard *fb, GPS *gps, IMU *imu);
double		inferBearing(FlightBoard *fb, GPS *gps, Logger *logs);

#endif
