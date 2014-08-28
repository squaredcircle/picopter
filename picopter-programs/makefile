CC = g++

CFLAGS = -Wall -c -g
INCLUDES = -I./include -I/usr/local/include -L/usr/local/lib
LDFLAGS = -lwiringPi -lboost_thread -lboost_system -lncurses -g

BASE = gpio logger flightBoard gps_qstarz imu_euler camera
BASE_INCLUDE = -I../picopter-base/include
BASE_HEADERS = $(addprefix ../picopter-base/include/, $(addsuffix .h, $(BASE)))
BASE_OBJECTS = $(addprefix ../picopter-base/obj/, $(addsuffix .o, $(BASE)))

XSENS_FLAGS =
XSENS_INCLUDES = -I../Xsens
XSENS_OBJECTS = $(addprefix ../Xsens/, cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o)

RASPICAM_FLAGS = -lraspicamcv -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host
RASPICAM_INCLUDES = -I../raspicam_cv -L../raspicam_cv -L../userland/build/lib
OPEN_CV = `pkg-config --cflags --libs opencv`

PROG = square saveWaypoints track_red_object snap_red_object
PROG_SOURCES = $(addprefix src/, $(addsuffix .cpp, $(PROG)))
PROG_OBJECTS = $(addprefix obj/, $(addsuffix .o, $(PROG)))
EXECUTABLES = $(addprefix bin/, $(PROG))


all: $(EXECUTABLES)


bin/square : obj/square.o $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_OBJECTS) $(RASPICAM_INCLUDES) $(RASPICAM_FLAGS) $(OPEN_CV) obj/square.o -o bin/square

bin/saveWaypoints : obj/saveWaypoints.o $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_OBJECTS) $(RASPICAM_INCLUDES) $(RASPICAM_FLAGS) $(OPEN_CV) obj/saveWaypoints.o -o bin/saveWaypoints

bin/track_red_object : obj/track_red_object.o $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_OBJECTS) $(RASPICAM_INCLUDES) $(RASPICAM_FLAGS) $(OPEN_CV) obj/track_red_object.o -o bin/track_red_object

bin/snap_red_object : obj/snap_red_object.o $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_OBJECTS) $(RASPICAM_INCLUDES) $(RASPICAM_FLAGS) $(OPEN_CV) obj/snap_red_object.o -o bin/snap_red_object



obj/square.o :  $(BASE_HEADERS) src/square.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) src/square.cpp -o obj/square.o

obj/saveWaypoints.o : $(BASE_HEADERS) src/saveWaypoints.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) src/saveWaypoints.cpp -o obj/saveWaypoints.o

obj/track_red_object.o : $(BASE_HEADERS) src/track_red_object.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) src/track_red_object.cpp -o obj/track_red_object.o

obj/snap_red_object.o : $(BASE_HEADERS) src/snap_red_object.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) src/snap_red_object.cpp -o obj/snap_red_object.o


clean:
	rm -f obj/*.o bin/* 
