//Author:	Omid Targhagh <20750454@student.uwa.edu.au>
//Date:		29-5-14
//Version:	v1.1
//
//Description:	Reads random burst of data from attached GPS
//Compile with flags : -Wall -Werror -lwiringPi

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
#include <wiringPi.h>

#define dataLength 500

int fileDes;
int gpsInt;
char gpsChar[8];
char gpsStr[256];

int main() {
	
	wiringPiSetup();
	
	fileDes = serialOpen("/dev/ttyACM0", 115200);	//Format serialOpen(char <device address>, int <baud rate>). May need to be changed.
	if (fileDes == -1) {
		printf("Error setting up GPS: %s\n", strerror(errno));
	}
	else {
		int gpsInt = 0;
		for (int i = 1; i <= dataLength; i++) {
			gpsInt = serialGetchar(fileDes); 	//Pulls data in
			sprintf(gpsChar, "%c", gpsInt);		//Converts data to string
			strcat(gpsStr, gpsChar);			//Clumps all the strings together
		}
		printf("GPS data is: \n%s\n", gpsStr);
	}
	serialClose(fileDes);
	printf("Ending...\n");
	return 0;
}

