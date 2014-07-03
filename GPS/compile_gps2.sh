#Author:	Omid Targhagh <20750454@student.uwa.edu.au>
#Date:		20-6-14
#Version:	v1

#Description:	Compiles all GPS related c++ files


echo compiling...

g++ -Wall -Werror	gpstest.cpp \
					gpsDataStream.h gpsDataStream.cpp \
					findComma.h findComma.cpp \
					-o gpstest\
					-I/usr/local/include -L/usr/local/lib -lwiringPi
					
echo gpstest compiled...
					
g++ -Wall -Werror	gpsRandomData.cpp \
					gpsDataStream.h gpsDataStream.cpp \
					-o gpsrandom\
					-I/usr/local/include -L/usr/local/lib -lwiringPi
					
echo getPosition compiled...

g++ -Wall -Werror	getPosition.cpp \
					gpsDataStream.h gpsDataStream.cpp \
					findComma.h findComma.cpp \
					-o gpsrandom\
					-I/usr/local/include -L/usr/local/lib -lwiringPi
					
echo getPosition compiled...

echo All done!
