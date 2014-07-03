//Author:	Omid Targhagh <20750454@student.uwa.edu.au>
//Date:		03-07-14
//Version:	v1
//
//Description:	Testing file for creating new functions
//Compile with flags : -Wall -Werror -lwiringPi

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <string>

#include "errno.h"
#include "wiringSerial.h"
#include "wiringPi.h"

#include "gpsDataStream.h"
#include "findComma.h"

#define header_length 6 	//First few characters in GPS data that determine it's type
#define lat_comma 2			//Postion of latitude data	
#define lon_comma 4			//Postion of longitude data	
#define dataLength 20		//Number of lines of GPS data to read
#define position "$GPGGA"	//Header for data stream containing location data	

int fileDes;
int gpsInt;
std::string gpsStr;
std::string gpsHeader;

extern int latStart;
extern int latLen;
extern int lonStart;
extern int lonLen;
int dummy;

struct gpsData 
{std::string lon;
 std::string lat;
}gpsData;


int main() {
	
	printf("Starting...\n");
	wiringPiSetup();
	printf("wiringPi has been set up\n");
	
	fileDes = serialOpen("/dev/ttyACM0", 115200);	//Format serialOpen(char <device address>, int <baud rate>). May need to be changed.
	if (fileDes == -1) {
		printf("Error setting up GPS. This may be due to:\n");
		printf("\t-A weak GPS signal\n");
		printf("\t-The incorrect Data Port being opened\n");
		printf("\t-Insufficient GPS Power\n");
	}
	else {
		printf("GPS initiated. Port value is: %i\n", fileDes);
		
		std::string gpsStr;
		std::string gpsHeader;
		gpsData.lat = "unknown";
		gpsData.lon = "unknown";
		
		for (int i = 0; i < dataLength; i++) {
			gpsStr = gpsDataStream(fileDes);	//Copies data from GPS into gpsStr
			//std::cout << gpsStr;
			gpsHeader = gpsStr.substr(0, header_length);
			if (gpsHeader.compare(position) == 0) {
				//std::cout << "Loop count: " << i << "\n";
				int latStart = findComma(gpsStr, lat_comma) + 1;
				int latLen = findComma(gpsStr, lat_comma + 1) - latStart;
				int lonStart = findComma(gpsStr, lon_comma) + 1;
				int lonLen = findComma(gpsStr, lon_comma + 1) - lonStart;
				
				gpsData.lat = gpsStr.substr(latStart, latLen);
				gpsData.lon = gpsStr.substr(lonStart, lonLen);
			}
		}
		std::cout << "Latitude: " << gpsData.lat << "\n";
		std::cout << "Longitude: " << gpsData.lon << "\n";
	}
	serialClose(fileDes);
	printf("Ending...\n");
	return 0;
}
