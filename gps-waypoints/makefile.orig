CC = g++

CFLAGS = -I. -I/usr/local/include -L/usr/local/lib -Wall -Werror -c -g
LDFLAGS = -lwiringPi -lboost_thread -g
SOURCES = gpio.cpp flightBoard.cpp gps_qstarz.cpp logger.cpp gps_util.cpp
HEADERS = $(SOURCES: .cpp = .h)
OBJECTS = $(SOURCES: .cpp = .o)

all : lawnmower waypoints2 waypoints saveWaypoints

lawnmower : lawnmower.o $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) lawnmower.o -o lawnmower

waypoints2 : waypoints2.o $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) waypoints2.o -o waypoints2

waypoints : waypoints.o $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) waypoints.o -o waypoints

saveWaypoints : saveWaypoints.o $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) saveWaypoints.o -o saveWaypoints

lawnmower.o : $(HEADERS) $(SOURCES)
	$(CC) $(CFLAGS) lawnmower.cpp

waypoints2.o : $(HEADERS) $(SOURCES)
	$(CC) $(CFLAGS) waypoints2.cpp

waypoints.o : $(HEADERS) $(SOURCES)
	$(CC) $(CFLAGS) waypoints.cpp

saveWaypoints.o : gps_qstarz.h gps_qstarz.cpp
	$(CC) $(CFLAGS) saveWaypoints.cpp

flightBoard.o : flightBoard.h flightBoard.cpp gpio.h gpio.cpp
	$(CC) $(CFLAGS) flightBoard.cpp

gps_qstarz.o : gps_qstarz.h gps_qstarz.cpp
	$(CC) $(CFLAGS) gps_qstarz.cpp

gpio.o : gpio.h gpio.cpp 
	$(CC) $(CFLAGS) gpio.cpp

logger.o : logger.h logger.cpp
	$(CC) $(CFLAGS) logger.cpp

clean:
	rm -f *.o 
	rm -f lawnmower waypoints2 waypoints
