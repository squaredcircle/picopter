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

#include "wiringSerial.h"
#include "wiringPi.h"

#include "gpsDataStream.h"
#include "findComma.h"

#define address "/dev/ttyACM0"	//Default address of Qstarz GPS.
//#define address "/dev/ttyUSB0"	//Default address of Piksi GPS.
#define baud_rate 115200		//Rate at which data read from GPS. Left at maximum
#define header_length 6 		//First few characters in GPS data that determine it's type
#define lat_comma 2				//Postion of latitude data	
#define lon_comma 4				//Postion of longitude data	
#define dataLength 20			//Number of lines of GPS data to read
#define position "$GPGGA"		//Header for data stream containing location data	

int fileDes;
int gpsInt;
std::string gpsStr;
std::string gpsHeader;

int latStart;
int latLen;
int lonStart;
int lonLen;
int dummy;

struct gpsData 
{std::string lon;
 std::string lat;
}gpsData;


int main() {
	
	wiringPiSetup();
	int fileDes = serialOpen(address, baud_rate);
	if (fileDes == -1) {
		std::cout << "Error setting up GPS. This may be due to:\n";
		std::cout << "\t-A weak GPS signal\n";
		std::cout << "\t-The incorrect Data Port being opened\n";
		std::cout << "\t-Insufficient GPS Power\n";
	}
	else {
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
	return 0;
}
