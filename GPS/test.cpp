//Author:	Omid Targhagh <20750454@student.uwa.edu.au>
//Date:		10-07-14
//
//Description:	Testing file for new GPS
//Compile with flags : -Wall -Werror

#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <string>

#include <ftdi.h>

#define vendorID 0x0403		//ID of chip vendor
#define productID 0x6001	//ID of chip

int open;
ftdi_context *context;

int main () {
	
	std::cout << "Starting...\n";
	context = ftdi_new();	//Creates new context structure for all ftdi functions
	open = ftdi_usb_open(context, vendorID, productID);	//Opens device
	
	if (open != 0) {			//Checks device opened correctly
		std::cout << "Error opening: got value " << open << " instead of 0.\n";
		return EXIT_FAILURE;
	}
	
	
	std::cout << "Ending...\n";
	return EXIT_SUCCESS;
}
