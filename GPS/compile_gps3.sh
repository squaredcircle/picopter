#Author:	Omid Targhagh <20750454@student.uwa.edu.au>
#Date:		10-07-14
#Version:	v1

#Description:	Compiles all GPS related C++ files for Piksi GPS


echo compiling...

g++ -Wall -Werror	test.cpp \
			-o gpstest\
			-I/usr/include -L/usr/lib/arm-linux-gnueabihf -libftdi1
echo All done!
