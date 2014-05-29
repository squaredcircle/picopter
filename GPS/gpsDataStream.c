//Author:	Omid Targhagh <20750454@student.uwa.edu.au>
//Date:		29-5-14
//Version:	v1.0
//
//Description:	Function that pulls out a line of data from an atached GPS
//Compile with flags : -Wall -Werror -lwiringPi

#include "gpsDataStream.h"


char* gpsDataStream(int fileDes, char* gpsStr) { //String passed as argument to avoid memory issues
	
	printf("Reading Data...\n");
	int finished = 0;		//C does not have boolean values as a class
	int gpsInt = serialGetchar(fileDes);
	char gpsChar[8];
	gpsStr[0] = '\0';
	
	while (gpsInt != '\n') {		//Waits for new-line charcter before starting to read
		gpsInt = serialGetchar(fileDes);
	}
	
	while (!finished) {
		gpsInt = serialGetchar(fileDes);
		sprintf(gpsChar, "%c", gpsInt);
		strcat(gpsStr, gpsChar);
		if (gpsInt == '\n') {		//found end of line
			finished = 1;
		}
	}
	return gpsStr;	
}
