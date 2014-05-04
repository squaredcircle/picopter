#!/bin/bash

#I was having trouble with the make file.  Feel free have a crack at it.

echo compiling...

g++ -Wall -Werror	flyInSquare1.cpp \
					flightBoardController.h flightBoardController.cpp \
					activationPinReader.h activationPinReader.cpp \
					-o flyInSquare1 \
					-I/usr/local/include -L/usr/local/lib -lwiringPi

echo done!
