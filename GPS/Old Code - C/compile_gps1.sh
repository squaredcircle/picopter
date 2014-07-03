#Authors:	Michael Baxter <20503664@student.uwa.edu.au> 
#			Omid Targhagh <20750454@student.uwa.edu.au>
#Date:		29-5-14
#Version:	v2.2

#Description:	Compiles all GPS related c files

#!/bin/bash
#I was having trouble with the make file.  Feel free have a crack at it.

echo compiling...

g++ -Wall -Werror	gpstest.c gpsDataStream.c \
					-o gpstest\
					-I/usr/local/include -L/usr/local/lib -lwiringPi
					
echo gpstest compiled...
					
g++ -Wall -Werror	gpsRandomData.c \
					-o gpsrandom\
					-I/usr/local/include -L/usr/local/lib -lwiringPi
					
echo gpsrandom compiled...

echo All done!
