//Compile with flags : -Wall -Werror -lwirngPi

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
#include </usr/local/include/wiringPi.h>

static int fileDes;
static int numberOfFields;
static int[] gpsData;

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
		if (numberOfFields != -1) {
			gpsData = (0, numberOfFields)
			for (int i = 0; i < count; ++i) {
				gpsData[i] = serialGetChar(fileDes);
			}
		}
	}
	
	return 0;
}
