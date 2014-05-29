//Author:	Omid Targhagh <20750454@student.uwa.edu.au>
//Date:		29-5-14
//Version:	v2.2
//
//Description:	Testing file for creating new functions
//Compile with flags : -Wall -Werror -lwiringPi

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
#include <wiringPi.h>

#include "gpsDataStream.h"

int fileDes;
int gpsInt;
char gpsChar[8];
char gpsStr[256];

int lon;
int lat;

struct gpsData 
{int lon;
 int lat;
}gpsData;

int main() {
	
	printf("Starting...\n");
	wiringPiSetup();
	printf("wiringPi has been set up\n");
	
	fileDes = serialOpen("/dev/ttyACM0", 115200);	//Format serialOpen(char <device address>, int <baud rate>). May need to be changed.
	if (fileDes == -1) {
		printf("Error setting up GPS: %s\n", strerror(errno));
	}
	else {
		printf("GPS initiated. Port value is: %i\n", fileDes);
		
		char gpsStr[256];
		
		for (int i = 0; i <10; i++) {
			printf("%s", gpsDataStream(fileDes, gpsStr));	//Last charcter already newline
		}
	}
	serialClose(fileDes);
	printf("Ending...\n");
	return 0;
}
