#include <stdlib.h>
#include <vector>
#include "lawnmower_structures.h"

bool usingIMU = true;
bool usingWindows = false;
std::vector<Pos> sweepEnds;

int SPEED_LIMIT = 35;		//Config file parameters - need to be initialised as globals
double SWEEP_SPACING = 7;
double POINT_SPACING = 7;
double WAYPOINT_RADIUS = 1.2;
double KPxy = 10;
double KIxy= 0.02;
/*double KPz =0;
double KIz = 0;
double DURATION = 0.5;
double FREQUENCY = 50;
int VOLUME = 50;*/
