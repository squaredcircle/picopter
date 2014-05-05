//Compile with flags : -Wall -Werror -lwiringPi

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
#include <wiringPi.h>

int fileDes;
int numberOfFields;
int gpsData[200];	//Arbitartly large size

int main() {
	
	printf("Starting...\n");
	wiringPiSetup();
	printf("wiringPi has been set up\n");
	
	fileDes = serialOpen("/dev/ttyAMA0", 115200);	//Format serialOpen(char <device address>, int <baud rate>). May need to be changed.
	if (fileDes == -1) {
		printf("Error setting up GPS: %s\n", strerror(errno));
	}
	else {
		printf("GPS initiated. Port value is: %i\n", fileDes);
		
		int numberOfFields = serialDataAvail(fileDes);	//Total number of dtat fields available. Returns -1 on error
		printf("Number of Fields is: %i\n", numberOfFields);
		if (numberOfFields != -1) {
			int gpsData[numberOfFields];
			for (int i = 0; i < numberOfFields; ++i) {
				gpsData[i] = serialGetchar(fileDes);
				printf("GPS location is: %i\n", gpsData[i]);
			}
		}
	}
	
	return 0;
}
