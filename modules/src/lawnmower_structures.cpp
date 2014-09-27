bool exitLawnmower = false;
bool usingIMU = true;

int SPEED_LIMIT = 35;		//Config file parameters - need to be initialised as globals
double SWEEP_SPACING = 6;
double POINT_SPACING = 3;
double WAYPOINT_RADIUS = 1.2;
double KPxy = 10;
double KIxy= 0;
double KPz = 0;
double KIz = 0;

int HMIN = 320;
int HMAX = 40;
int SMIN=  95;
int SMAX = 255;
int VMINIMUM = 95;
int VMAX = 255;
int WHITE = 255;
int BLACK = 0;
int COLSIZE = 160;
int ROWSIZE = 120; 
int PIXELTHRESH = 12;
int DILATE_ELEMENT = 6;
int ERODE_ELEMENT = 6;