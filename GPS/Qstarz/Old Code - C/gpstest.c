//Author:	Omid Targhagh <20750454@student.uwa.edu.au>
//Date:		29-5-14
//Version:	v2.3
//
//Description:	Testing file for creating new functions
//Compile with flags : -Wall -Werror -lwiringPi

#define header_length 6 	//First few characters in GSP data that determine it's type

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

char gpsHeader[8*header_length]; //6 charcters long, each 8 bits

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
		char gpsHeader[8*header_length];
		
		for (int i = 0; i <10; i++) {
			printf("%s", gpsDataStream(fileDes, gpsStr));	//Copies data from GPS into gpsStr
			for (int j = 0; j < header_length; j++) {		//Extraxts first few chars to determine type
				gpsHeader[j] = gpsStr[j];
			}
			if (strcmp(gpsHeader, "$GPGGA") == 0) {
				sprintf(gpsData.lat, substr(gpsStr, 18, 9));
				sprintf(gpsData.lon, substr(gpsStr, 30, 10));
			}
		}
	}
	serialClose(fileDes);
	printf("Ending...\n");
	return 0;
}
