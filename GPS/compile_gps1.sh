#!/bin/bash

#I was having trouble with the make file.  Feel free have a crack at it.

echo compiling...

g++ -Wall -Werror	gpstest.c \
					-o gpstest\
					-I/usr/local/include -L/usr/local/lib -lwiringPi

echo done!
