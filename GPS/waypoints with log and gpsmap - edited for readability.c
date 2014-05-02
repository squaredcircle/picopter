/*
// Chris Venables -- University of Western Australia -- 1/09/13
// This code is designed to allow the hexacopter to drive GPS waypoints.
// It is theoretically designed to fly GPS waypoints without a compass required
// This also uses the servoblaster module for accurate pulse width modulation output signals
// It is currently set up to be able to fly up to 10 waypoints however this can easily be scaled up or down
 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
#include <math.h>
#include <time.h>
// Added for GPIO interaction using wiringPi
#include </usr/local/include/wiringPi.h>

#define PI 3.14159265358979323846
#define earth_radius 6371000	//in meters

#define number_waypoints 6			//Must be a number between 1 and 10 inclusive
#define waypoint1_lat 31.9797		//Fill this with the desired waypoints
#define waypoint1_long 115.8177	//Give waypoint as decimal degrees
#define waypoint2_lat 31.9803
#define waypoint2_long 115.8177
#define waypoint3_lat 31.9800
#define waypoint3_long 115.8181
#define waypoint4_lat 31.9797
#define waypoint4_long 115.8177
#define waypoint5_lat	31.9803	//All waypoints must be given values even if they are not going to be used
#define waypoint5_long	115.8177
#define waypoint6_lat	31.9800
#define waypoint6_long	115.8181

#define waypoint7_lat	30.0000
#define waypoint7_long	115.0000
#define waypoint8_lat	30.0000
#define waypoint8_long	115.0000
#define waypoint9_lat	30.0000
#define waypoint9_long	115.0000
#define waypoint10_lat	30.0000
#define waypoint10_long	115.0000


// Define Steering GPIO Ports (according to servoblaster numbering)
#define Elevator 1
#define Aileron 0
// Define Activation switch GPIO Ports (according to wiringPi)
#define Activate 5

// Define parameters for PWM
#define minpwm 110
#define maxpwm 194
#define steer 0.125


static int fd;
static int firsttime = 1;
static int auto_control = 1;		//static int auto_control = 0;
static int restart = 1;
static char file_string [256]; //to store log file name
static char logstr [1024];
static char log_string;
//static int firstgpsdata = 1;
static char gpsstr[256];
static char gpschar[8];
static char gpstimestr[10];
static char gpsstatus[1];
static char gpslatstr_hh[2];
static char gpslatstr_mm[7];
static char gpslatstrhemi[1];
static char gpslongstr_hh[3];
static char gpslongstr_mm[7];
static char gpslongstrhemi[1];
static char gpsspeedstr[4];
static char gpsbearingstr[6];
static char gpsdatestr[6];
static double gpstime;
static double gpslat;
static double gpslat_hh;
static double gpslat_mm;
static double gpslong;
static double gpslong_hh;
static double gpslong_mm;
static double gpsspeed;
static double gpsbearing;
static double gpsdate;
static double leftright;
static double frontback;
static double direction;
static double target_bearing_rel;
static double waypoint_lat;
static double waypoint_long;



int gpsdata(void) {	//Populates the gps data fields with updated date then returns
  for(;;) {
		int gpsint = serialGetchar(fd);
		if (gpsint < 0) {
		    printf("Error reading serial data: %s\n", strerror (errno)) ;
			sprintf(logstr,"Error reading serial data: %s\n", strerror (errno));
			if(0!=log_file(logstr)) fail(logstr);
		    return 1 ;
		}
	
		sprintf(gpschar, "%c\0", gpsint);
		strcat(gpsstr, gpschar);
		if(gpsint == 10) {
			if(0 == strncmp(gpsstr, "$GPRMC", 6)) {	//If gps character is end of line character
										//Check for correct GPS data format line
			//if(firstgpsdata)
			//{
				sprintf(logstr,"GPS data string obtained = %s\n", gpsstr);
				if(0!=log_file(logstr)) fail(logstr);

				printf("%s",gpsstr);		//Display GPS data string the first time only
				//firstgpsdata = 0;
				//}
				//Then read data to strings
				int prevcommapos = 0;
				int strcount = 0;
				int charcount;
				for(charcount = 0; charcount < 256 && gpsstr[charcount] != 10; charcount++) {
					if(gpsstr[charcount] == ',' && strcount < 10) {
						switch(strcount) {		// This pulls out each component of the GPS data based on comma position
											// Note this currently assumes all fields have data
						case 1:
							strncpy(gpstimestr, gpsstr + (prevcommapos + 1), charcount - (prevcommapos + 1));
							gpstimestr[10]='\0';
							sscanf(gpstimestr, "%lf", &gpstime);
							//printf("GPS Time = %.3lf\n", gpstime);
							break;
						
						case 2:
							strncpy(gpsstatus, gpsstr + (prevcommapos + 1), charcount - (prevcommapos + 1));
							gpsstatus[1]='\0';
							//printf("GPS Status = %s\n",gpsstatus);
							break;
					
						case 3:
							if(charcount - prevcommapos > 7)	//Check if latitude data is filled
							{
								strncpy(gpslatstr_hh, gpsstr + (prevcommapos + 1), (charcount - 8));
								gpslatstr_hh[2]='\0';	
								sscanf(gpslatstr_hh, "%lf", &gpslat_hh);
								strncpy(gpslatstr_mm, gpsstr + (charcount - 7), charcount);	
								gpslatstr_mm[7]='\0';	
								sscanf(gpslatstr_mm, "%lf", &gpslat_mm);
								gpslat = gpslat_hh + gpslat_mm/60;			//Get gps lat data in decimal degrees
								//printf("GPS Latitude = %.4lf\n", gpslat);
							}
							else	//Else if no latitude data
							{
								gpslat = 0;
								printf("No GPS Latitude data available\n");
								sprintf(logstr,"No GPS Latitude = data available\n");
								if(0!=log_file(logstr)){
								fail(logstr);}
							}	
								break;
							
						case 4:
							strncpy(gpslatstrhemi, gpsstr + (prevcommapos + 1), charcount - (prevcommapos + 1));
							gpslatstrhemi[1]='\0';
							//printf("GPS Latitude Hemisphere = %s\n",gpslatstrhemi);
							break;
							
						case 5:
							if(charcount - prevcommapos > 7)	//Check if longitude data is available
							{
								strncpy(gpslongstr_hh, gpsstr + (prevcommapos + 1), (charcount - 8));
								gpslongstr_hh[3]='\0';
								sscanf(gpslongstr_hh, "%lf", &gpslong_hh);
								strncpy(gpslongstr_mm, gpsstr + (charcount - 7), charcount);
								gpslongstr_mm[7]='\0';
								sscanf(gpslongstr_mm, "%lf", &gpslong_mm);
								gpslong = gpslong_hh + gpslong_mm/60;			//Get gps lat data in decimal degrees
								//printf("GPS Longitude = %.4lf\n", gpslong);
							}
							else	//Else if no longitude data
							{
								gpslong = 0;
								printf("No GPS Latitude data available\n");
								sprintf(logstr,"No GPS Latitude = data available\n");
								if(0!=log_file(logstr)){
								fail(logstr);}
							}
							break;
							
						case 6:
							strncpy(gpslongstrhemi, gpsstr + (prevcommapos + 1), charcount - (prevcommapos + 1));
							gpslongstrhemi[1]='\0';
							//printf("GPS Longitude Hemisphere = %s\n",gpslongstrhemi);
							break;
							
						case 7:
							strncpy(gpsspeedstr, gpsstr + (prevcommapos + 1), charcount - (prevcommapos + 1));
							gpsspeedstr[4]='\0';
							sscanf(gpsspeedstr, "%lf", &gpsspeed);
							//printf("GPS Speed = %.2lf\n", gpsspeed);
							break;
							
						case 8:
							strncpy(gpsbearingstr, gpsstr + (prevcommapos + 1), charcount - (prevcommapos + 1));
							gpsbearingstr[6]='\0';
							sscanf(gpsbearingstr, "%lf", &gpsbearing);
							//printf("GPS Bearing = %.2lf\n", gpsbearing);
							break;
							
						case 9:
							strncpy(gpsdatestr, gpsstr + (prevcommapos + 1), charcount - (prevcommapos + 1));
							gpsdatestr[6]='\0';
							sscanf(gpsdatestr, "%lf", &gpsdate);
							//printf("GPS Date = %.lf\n", gpsdate);
							break;
						}
					prevcommapos = charcount;
					strcount++;
					}
				}
		
				// printf("GPS Time = %.3lf\n", gpstime);
				// printf("GPS Status = %s\n",gpsstatus);
				printf("GPS Latitude = %.4lf\n", gpslat);
				// printf("GPS Latitude Hemisphere = %s\n",gpslatstrhemi);
				printf("GPS Longitude = %.4lf\n", gpslong);
				// printf("GPS Longitude Hemisphere = %s\n",gpslongstrhemi);
				// printf("GPS Speed = %.2lf\n", gpsspeed);
				printf("GPS Bearing = %.2lf\n", gpsbearing);
				// printf("GPS Date = %.lf\n", gpsdate);
			
				sprintf(logstr,"GPS Time = %.3lf\n", gpstime);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}
				sprintf(logstr,"GPS Status = %s\n",gpsstatus);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}			
				sprintf(logstr,"GPS Latitude = %.4lf\n", gpslat);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}			
				sprintf(logstr,"GPS Latitude Hemisphere = %s\n",gpslatstrhemi);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}			
				sprintf(logstr,"GPS Longitude = %.4lf\n", gpslong);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}			
				sprintf(logstr,"GPS Longitude Hemisphere = %s\n",gpslongstrhemi);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}					
				sprintf(logstr,"GPS Speed = %.2lf\n", gpsspeed);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}					
				sprintf(logstr,"GPS Bearing = %.2lf\n", gpsbearing);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}					
				sprintf(logstr,"GPS Date = %.lf\n", gpsdate);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}		
			gpsstr[0] = '\0';
			break;
			}
		gpsstr[0] = '\0';	//If not correct GPS data format, clear string and get next line.
		}
	}
	return 0;
}


double determine_bearing(int waypointvar) {
	gpsdata();
	direction = gpsbearing;
	sprintf(logstr,"Determine current bearing\n");
	if(0!=log_file(logstr)){
	fail(logstr);}

	switch(waypointvar) {
		case 1:
			printf("This is for waypoint 1\n");
			waypoint_lat = waypoint1_lat;
			waypoint_long = waypoint1_long;
			break;
		case 2:
			printf("This is for waypoint 2\n");
			waypoint_lat = waypoint2_lat;
			waypoint_long = waypoint2_long;
			break;
		case 3:
			printf("This is for waypoint 3\n");
			waypoint_lat = waypoint3_lat;
			waypoint_long = waypoint3_long;
			break;
		case 4:
			printf("This is for waypoint 4\n");
			waypoint_lat = waypoint4_lat;
			waypoint_long = waypoint4_long;
			break;
		case 5:
			printf("This is for waypoint 5\n");
			waypoint_lat = waypoint5_lat;
			waypoint_long = waypoint5_long;
			break;
		case 6:
			printf("This is for waypoint 6\n");
			waypoint_lat = waypoint6_lat;
			waypoint_long = waypoint6_long;
			break;
		case 7:
			printf("This is for waypoint 7\n");
			waypoint_lat = waypoint7_lat;
			waypoint_long = waypoint7_long;
			break;
		case 8:
			printf("This is for waypoint 8\n");
			waypoint_lat = waypoint8_lat;
			waypoint_long = waypoint8_long;
			break;
		case 9:
			printf("This is for waypoint 9\n");
			waypoint_lat = waypoint9_lat;
			waypoint_long = waypoint9_long;
			break;
		case 10:
			printf("This is for waypoint 10\n");
			waypoint_lat = waypoint10_lat;
			waypoint_long = waypoint10_long;
			break;
	}

	sprintf(logstr,"Current waypoint = %d\n", waypointvar);
	if(0!=log_file(logstr)){
	fail(logstr);}

	double delta_lat = waypoint_lat - gpslat;		//This is calculated in decimal degrees
	double delta_long = waypoint_long - gpslong;

	sprintf(logstr,"Delta Latitude = %lf\nDelta Longitude = %lf\n", delta_lat, delta_long);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}

	double required_bearing = 180/PI * atan(delta_long/delta_lat);	//required absolute bearing from hexacopter to target waypoint in degrees

	if(delta_lat * delta_long < 0)	{	//Convert value between -90 and 90 degrees to a value between 0 and 360 degrees
		required_bearing = required_bearing + 180;
	}
	if(delta_long < 0) {
		required_bearing = required_bearing + 180;
	}

	printf("Required absolute true bearing = %.2lf\n", required_bearing);
		sprintf(logstr,"Required absolute bearing to target = %lf\n", required_bearing);
		if(0!=log_file(logstr)) {
			fail(logstr);
		}
			
	double rel_orient = atan(leftright/frontback);	//Relative orientation of the front of the hexacopter relative to its intended direction of travel
	if(leftright * frontback > 0) { 	//Convert value between -90 and 90 degrees to a value between 0 and 360 degrees
		rel_orient = rel_orient + 180;
	}
	if(leftright > 0) {
		rel_orient = rel_orient + 180;
	}

	sprintf(logstr,"Relative orientation of hexacopter front to previous motion direction = %lf\n", rel_orient);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}

	double abs_orient = direction + rel_orient;		//Absolute orientation of the front of the hexacopter relative to North as determined using GPS bearing info

	while(abs_orient > 360)	{	//Ensure abs_orient is within 0 and 360 degrees
		abs_orient = abs_orient - 360;
	}

	while(abs_orient < 0) {
		abs_orient = abs_orient + 360;
	}

	printf("Absolute hexacopter orientation bearing = %.2lf\n", abs_orient);
	sprintf(logstr,"Absolute orientation of hexacopter = %lf\n", abs_orient);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}

	double target_bearing_rel_var = 360 - abs_orient + required_bearing;	//target bearing relative to absolute orientation of front of craft. This governs how to steer.

	printf("Target bearing relative to hexacopter orientation = %.2lf\n", target_bearing_rel_var);
	sprintf(logstr,"Target bearing relative to hexacopter orientation = %lf\n", target_bearing_rel_var);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}

	return (target_bearing_rel_var);
}


double distance(void) {	//Check distance from current location to target waypoint		//This uses the Haversine formula
	gpsdata();	//update gps data
	double delta_lat_var = waypoint_lat - gpslat;
	double delta_long_var = waypoint_long - gpslong;
	sprintf(logstr,"Distance calc Delta Latitude = %lf\nDistance calc Delta Longitude\n", delta_lat_var, delta_long_var);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}
	printf("Delta latitude = %lf. Delta longitude = %lf.\n", delta_lat_var, delta_long_var);
	//Note the trig functions must have rad inputs
	double distvar1 = sin(PI/180*delta_lat_var/2)*sin(PI/180*delta_lat_var/2) + cos(PI/180*gpslat)*cos(PI/180*waypoint_lat)*sin(PI/180*delta_long_var/2)*sin(PI/180*delta_long_var/2);
	double distvar2 = 2*atan2(sqrt(distvar1), sqrt(1-distvar1));
	double distance = earth_radius * distvar2;

	printf("a = %lf\nc = %lf\n", distvar1, distvar2);

	printf("Straight line distance to target = %.2lf meters\n", distance);
	sprintf(logstr,"Straight line distance to target = %.2lf meters\n", distance);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}
				
	//Second calculation method
	// long double delta_long_var2 = delta_long_var;
	// long double delta_lat_var2 = delta_lat_var;
	// long double distvar11 = sin(PI/180*delta_long_var2/2)*sin(PI/180*delta_long_var/2);
	// long double distvar22 = cos(PI/180*gpslat)*cos(PI/180*waypoint_lat)*distvar11;
	// long double distvar33 = sin(PI/180*delta_lat_var2/2)*sin(PI/180*delta_lat_var2/2);
	// long double distvar44 = distvar33 - distvar22;
	// long double distvar55 = sqrt(distvar44);
	// long double distvar66 = asin(distvar55);
	// long double distance2 = 2 * earth_radius * distvar66;
	// printf("1 = %Lf, 2 = %Lf, 3 = %Lf, 4 = %Lf, 5 = %Lf, 6 = %Lf\n", distvar11, distvar22, distvar33, distvar44, distvar55, distvar66);
	// printf("distance2 = %Lf\n", distance2);
						
	return (distance);	//return distance in meters
}

int rest(void) {	//Stable hover when called. Allows craft to rest while other calculations are performed.
	char servopos[128];
	int stable = (maxpwm+minpwm)/2;
	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, stable);
	system(servopos);
	sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, stable);
	system(servopos);
	
	printf("Currently resting. Awaiting further instruction.\n");

	return 0;
}

int fly(double theta) {	//theta is between 0 and 360 degrees and governs the direction the vehicle flys

	double rad_theta = theta * PI/180;
	leftright = sin(rad_theta);	//this angle must be in radians
	frontback = cos(rad_theta);
	sprintf(logstr,"Fly -- sin(theta) = %lf\nFly -- cos(theta) = %lf\n", leftright, frontback);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}
	double lr_var = (maxpwm+minpwm)/2 + steer*(maxpwm-minpwm)*leftright;
	double fb_var = (maxpwm+minpwm)/2 - steer*(maxpwm-minpwm)*frontback;
	sprintf(logstr,"Accurate steering value lr = %lf\nAccurate steering value fb = %lf\n", lr_var, fb_var);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}
	int lr = round(lr_var);	//adjust to vary correctly
	int fb = round(fb_var);	//only an int because that's all that servoblaster accepts
							//fb is reversed??
	sprintf(logstr,"Naza input value lr = %d\nNaza input value fb = %fb\n", lr, fb);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}
				
	char servopos[128];			
	if(gpslat * gpslong != 0) {	//Check if valid GPS data
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, lr);
		system(servopos);
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, fb);
		system(servopos);

		printf("Fly at %.2lf degrees. Left/right = %d. Front/back = %d.\n", theta, lr, fb);
	}
	else {	//If no valid GPS data just hover
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, (minpwm+maxpwm)/2);
		system(servopos);
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, (minpwm+maxpwm)/2);
		system(servopos);

		printf("Fly at %.2lf degrees. Left/right = %d. Front/back = %d.\n", theta, (minpwm+maxpwm)/2, (minpwm+maxpwm)/2);
	}
	return 0;
}

int recorrect(int waypointint) {	//This function recorrects flight bearing to account for wind or gps errors
	//double old_target_bearing_rel = target_bearing_rel;
	printf("\nRecorrecting flight path to waypoint %d\n", waypointint);
	sprintf(logstr,"\nRecorrecting flight path to waypoint %d\n", waypointint);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}	
	double target_bearing_rel = determine_bearing(waypointint);
	//double bearing_error = target_bearing_rel - old_target_bearing_rel;
	//target_bearing_rel = 
	fly(target_bearing_rel);
	usleep(3000000);

	return 0;
}

int waypoint(int waypointnum) {
	printf("Fly towards target waypoint %d\n", waypointnum);
	target_bearing_rel = determine_bearing(waypointnum);
	fly(target_bearing_rel);	//fly towards target waypoint

	int time_old = millis();			//Get new time in milliseconds
	for(;;) {
		double dist = distance();
		if(dist < 10) {		//check if within 10 meters of target
			break;
		}
		sprintf(logstr,"Waypoint not yet reached\n");
		if(0!=log_file(logstr)) {
			fail(logstr);
		}	
		int time_new = millis();
		int delta_time = time_new - time_old;
		if(delta_time > 3000) {	//Check if have been flying for more than 3 seconds, if so recorrect flight bearing
			int time_check_decider = 0;
			while(distance() > 10) {
				sprintf(logstr,"Waypoint %d not yet reached\n", waypointnum);
				if(0!=log_file(logstr)) {
					fail(logstr);
				}
				
				printf("Waypoint %d not yet reached.\n", waypointnum);
				recorrect(waypointnum);
				
				if(time_check_decider++ > 10) {	//Only check time every 10th cycle
					activate_switch();
					if(auto_control == 0) {
							sprintf(logstr,"Activate switch off. Restart program.\n");
							if(0!=log_file(logstr)) {
								fail(logstr);
							}
							restart = 1;
							break;
					}
					int time_new = millis();
					int delta_time = time_new - time_old;
					if(delta_time > 180000) {	//If have spent 3 minutes seeking waypoint unsuccessfully, quit.
						printf("Waypoint %d could not be found. Stopping program immediately.\n", waypointnum);
						sprintf(logstr,"Waypoint %d could not be found. Stopping program immediately.\n", waypointnum);
						if(0!=log_file(logstr)) {
							fail(logstr);
						}
						return 1;
					}
					time_check_decider = 0;
				}
			}
			break;
		}
	}

	if(restart) {
	printf("Waypoint program restarting\n");
	usleep(1000000);
	}
	else {	
	printf("Waypoint %d reached. Hovering for 10 seconds.\n", waypointnum);
	sprintf(logstr,"Waypoint %d reached. Hovering for 10 seconds.\n", waypointnum);
	if(0!=log_file(logstr)) {
		fail(logstr);
	}
	rest();
	usleep(15000000);
	}

	return 0;
}

int activate_switch() {	//Check activation switch status. If off wait.
	/*
	if(firsttime) {
		if(!digitalRead(Activate)) {
			firsttime=0;
		}
		auto_control = 0;
		char servopos[128];
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, (minpwm+maxpwm)/2);
		system(servopos);
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, (minpwm+maxpwm)/2);
		system(servopos);

		printf("Autonomous program off. Waiting for on switch\n");
		sprintf(logstr,"Autonomous program off. Waiting for on switch\n");
		if(0!=log_file(logstr)){
		fail(logstr);}
	}		
	else if(digitalRead(Activate) == 0) {	//If activate switch is low just stable hover
		auto_control = 0;
		char servopos[128];
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, (minpwm+maxpwm)/2);
		system(servopos);
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, (minpwm+maxpwm)/2);
		system(servopos);

		printf("Autonomous program off. Waiting for on switch\n");
			sprintf(logstr,"Autonomous program off. Waiting for on switch\n");
		if(0!=log_file(logstr)) {
			fail(logstr);
		}
	}

	else if(digitalRead(Activate) == 1 && auto_control == 0) {
		//If autonomous mode only just turned on, continue stable hover for 2 seconds
		auto_control = 1;
		printf("Autonomous program engaged\n");
		sprintf(logstr,"Autonomous program engaged\n");
		if(0!=log_file(logstr)) {
			fail(logstr);
		}
		usleep(2000000);	//usleep
		firsttime = 0;
	}
	else(auto_control = 1);
	*/
	return 0;
}


int log_file(log_string) {	//function to add info to log file
	FILE *fp;    /* File pointer */
	   	/* Open the log file for writing */
	   	if (NULL == (fp = fopen(file_string,"a"))) {
	      	printf("Error printing to text log file\n"); // if writing fails print error and exit
	    	return 1;
	   	}
	   
	char print_log_str[1024];
	sprintf(print_log_str,"%s",log_string);
	
	fprintf(fp, print_log_str);  /* write the provided string to the file */

	   
	if (!0 == fclose(fp)){ /* close the file we opened earlier*/
		printf("Error closing text log file\n");
		return 1;
	   }
	return 0;
}

int create_log_file(void) {
	//Create Log File
	printf("\nCreate fault log file\n");
 	struct tm *time_data ;	//Get time data
  	time_t rawtime;
  	char time_string [128] ;

  	rawtime = time (NULL) ;
  	time_data = localtime(&rawtime) ;  
  	strftime(time_string,128,"%d-%b-%Y %H-%M-%S",time_data);  	//Store current time data in string


	FILE *fp;    /* File pointer */

	sprintf(file_string, "/home/pi/Flight_Log_Files/Waypoint Log File -- %s.txt",time_string);	//Create log file name

	int filenumber = 1;
	while(NULL != (fp = fopen(file_string,"r")) && filenumber < 10) { //check if filename is already taken
		fclose(fp);
		sprintf(file_string,"%s#",file_string);		//if name is taken append a "#"
		filenumber++;
	}
	if(filenumber > 10) {					//if after 10 "#" have been appended and name is still taken, exit
		sprintf(logstr,"Error naming text log\n");
		printf("%s",logstr);	// print error and exit
		fail(logstr);
	}

	printf("Log file location = %s\n",file_string);

	sprintf(logstr,"Flight Log File\n%s\n\n", time_string);//Initialise log file
	if(0!=log_file(logstr)) {	//exit if log file printing function fails
		fail(logstr);
	}
	
	return 0;
}

int fail(fail_str) {	//This function is called if the system fails in some way
					//Print error message and maintain stable hover forever
	printf("*#*#*#*#*#*#*#*#*#*#*\n\n*#*#*#*#*#*#*#*#*#*#*\n\nFault has occured\nObject tracking deactivated\n\n*#*#*#*#*#*#*#*#*#*#*\n\n*#*#*#*#*#*#*#*#*#*#*\n");
	printf("Fail string = '%s'\n",fail_str);
	for(;;){
		char servopos[128];
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, (minpwm+maxpwm)/2);
		system(servopos);
		sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, (minpwm+maxpwm)/2);
		system(servopos);
		usleep(500000);
	}
	return 0;
}

//Put these at the top
int createmap = 1;
double oldgpslat;
double oldgpslong;
double oldgpsbearing;

/*
int gpsmap(void) {
if(createmap) {
	IplImage* imgHSV = cvCreateImage(cvGetSize(dstImage), 8, 3);	//create image file
	//make sure image is all black or all white or something
	return 0;
}
*/

int main () {
	printf("GPS Waypoint Test\n");

	//Start WiringPi
	wiringPiSetup();

	// Start Servoblaster servod
	system("/home/pi/PiBits/ServoBlaster/servod");	//put in here a check for servod status or failure

	//Initialise GPS
  	if ((fd = serialOpen ("/dev/ttyACM0", 115200)) < 0) {
    	printf("Unable to open serial device: %s\n", strerror (errno)) ;
    	return 1 ;
  	}

	create_log_file();
	  
	printf("All setup parameters initialised correctly\n");

	sprintf(logstr,"All setup parameters initialised correctly\n");
	if(0!=log_file(logstr)) {
		fail(logstr);
	}

	for(;;) {
		restart = 0;
		printf("Restarting waypoint program\n");
		sprintf(logstr,"Restarting waypoint program\n");
		if(0!=log_file(logstr)) {
			fail(logstr);
		}
			
		for(;;) { 
			//Halt until program activated for the first time
			if(activate_switch()!=0) {	//Check activation switch status.
				printf("Activation switch check error\n");
				sprintf(logstr,"Activation switch check error\n");
				if(0!=log_file(logstr)){
					fail(logstr);
				}
			}
			usleep(1000000);
			
			if(auto_control) {	//only steer if autonomy is activated
				printf("Waypoint program activated\n");
				sprintf(logstr,"Waypoint program activated\n");
				if(0!=log_file(logstr)) {
					fail(logstr);
				}
				direction = 0;	//Assume facing directly north
				target_bearing_rel = direction;
				printf("Fly forwards to initialise bearing and orientation data\n");
				fly(target_bearing_rel);
				usleep(5000000);	//Fly straight for 3 seconds to get bearing
				rest();		//Hover while other calculations are done

				//Start waypoints
				int waypointcounter;
				for(waypointcounter = 1; waypointcounter <= number_waypoints; waypointcounter++) {
					sprintf(logstr,"New waypoint = Waypoint %d\n", waypointcounter);
					if(0!=log_file(logstr)) {
						fail(logstr);
					}
				
					if(!(0 == waypoint(waypointcounter))) {
						printf("Error attempting to locate GPS target waypoint.\nProgram exiting immediately!\n");
						sprintf(logstr,"Error attempting to locate GPS target waypoint.\nProgram exiting immediately!\n");
						if(0!=log_file(logstr)) {
							fail(logstr);
						}
					}
					if(restart) {
						break;
					}
				}
				if(restart) {
					break;
				}
				printf("Final waypoint reached. Awaiting further instruction.\n");
				sprintf(logstr,"Final waypoint reached. Awaiting further instruction\n");
				if(0!=log_file(logstr)) {
					fail(logstr);
				}
				for(;;) {
					rest();
					usleep(2000000);
					sprintf(logstr,"Final waypoint reached. Awaiting further instruction\n");
					if(0!=log_file(logstr)) {
						fail(logstr);
					}
				}
			}
			else {	//If activate switch is off just send hover signal (even though it does nothing)
				gpsdata();	//Display current data
				sprintf(logstr,"Activation switch off. Hover in place.\n");
				if(0!=log_file(logstr)) {
					fail(logstr);
				}
				int lr = (minpwm + maxpwm)/2;
				int fb = (minpwm + maxpwm)/2;
				char servopos[128];
				sprintf(servopos, "echo %d=%d > /dev/servoblaster",Aileron, lr);
				system(servopos);
				sprintf(servopos, "echo %d=%d > /dev/servoblaster",Elevator, fb);
				system(servopos);
			}		
		}
	}
	return 0;
}



