#ifndef __LAWNMOWER_STRUCTURES_INCLUDED__
#define __LAWNMOWER_STRUCTURES_INCLUDED__

typedef struct{		//These are in degrees now.
	double lat;
	double lon;
} Pos;

extern bool exitLawnmower;
extern bool usingIMU;

extern int SPEED_LIMIT;		//Config file parameters - need to be initialised as globals
extern double SWEEP_SPACING;
extern double POINT_SPACING;
extern double WAYPOINT_RADIUS;
extern double KPxy;
extern double KIxy;
extern double KPz;
extern double KIz;

extern int HMIN;
extern int HMAX;
extern int SMIN;
extern int SMAX;
extern int VMINIMUM;
extern int VMAX;
extern int WHITE;
extern int BLACK;
extern int COLSIZE;
extern int ROWSIZE; 
extern int PIXELTHRESH;
extern int DILATE_ELEMENT;
extern int ERODE_ELEMENT;

#endif
