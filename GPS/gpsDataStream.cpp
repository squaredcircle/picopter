//Author:	Omid Targhagh <20750454@student.uwa.edu.au>
//Date:		20-6-14
//Version:	v1.2
//
//Description:	Function that pulls out a line of data from an atached GPS
//Compile with flags : -Wall -Werror -lwiringPi

#include "gpsDataStream.h"

std::string dataStr;
int finished;

std::string gpsDataStream(int fileDes) {
	
	int finished = 0;		//C does not have boolean values as a class
	int gpsInt = serialGetchar(fileDes);
	std::string dataStr = "";
	
	while (gpsInt != '\n') {		//Waits for new-line charcter before starting to read
		gpsInt = serialGetchar(fileDes);
	}
	
	while (!finished) {
		gpsInt = serialGetchar(fileDes);
		dataStr += gpsInt;			//Appends char from GPS to string
		if (gpsInt == '\n') {		//Found end of line
			finished = 1;
		}
	}
	return dataStr;	
}
