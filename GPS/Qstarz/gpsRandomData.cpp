//Author:	Omid Targhagh <20750454@student.uwa.edu.au>
//Date:		20-6-14
//Version:	v1.2
//
//Description:	Reads random burst of data from attached GPS
//Compile with flags : -Wall -Werror -lwiringPi

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <string>

#include "wiringSerial.h"
#include "wiringPi.h"
#include "gpsDataStream.h"

#define dataLength 20

int fileDes;
int gpsInt;

int main() {
	
	wiringPiSetup();
	
	fileDes = serialOpen("/dev/ttyUSB0", 1000000);	//Format serialOpen(char <device address>, int <baud rate>). May need to be changed.
	if (fileDes == -1) {
		std::cout << "Error setting up GPS. This may be due to:\n";
		std::cout << "\t-A weak GPS signal\n";
		std::cout << "\t-The incorrect Data Port being opened\n";
		std::cout << "\t-Insufficient GPS Power\n";
	}
	else {
		std::string gpsStr;
		std::cout << "GPS data is: \n";
		for (int i = 1; i <= dataLength; i++) {
			std::cout << gpsDataStream(fileDes);
		}
	}
	serialClose(fileDes);
	std::cout << "Ending...\n";
	return 0;
}

