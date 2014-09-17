##### General
CC                = g++
CFLAGS            = -O2 -Wall -c -g
INCLUDES          = -I./include -I/usr/local/include 
LIBS              = -L/usr/local/lib -L/usr/lib
LDFLAGS           = -lwiringPi -lthrift -lboost_thread -lboost_system -lncurses -g

##### Thrift
THRIFT_FLAGS      = -DHAVE_INTTYPES_H -DHAVE_NETINET_IN_H 
THRIFT_SRC        = gen-cpp/picopter_types.cpp gen-cpp/webInterface.cpp
THRIFT_INC        = -Igen-cpp -I/usr/local/include/thrift

##### picopter-base
BASE              = gpio logger flightBoard gps_qstarz 
#camera imu_euler 
BASE_INCLUDE      = -I../picopter-base/include
BASE_HEADERS      = $(addprefix ../picopter-base/include/, $(addsuffix .h, $(BASE)))
BASE_OBJECTS      = $(addprefix ../picopter-base/obj/, $(addsuffix .o, $(BASE)))

##### XSens
#XSENS_FLAGS       =
#XSENS_INCLUDES    = -I../Xsens
#XSENS_OBJECTS     = $(addprefix ../Xsens/, cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o)

##### Raspicam, OpenCV
#RASPICAM_FLAGS    = -lraspicamcv -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host
#RASPICAM_INCLUDES = -I../../robidouille/raspicam_cv -L../../robidouille/raspicam_cv -L../../userland/build/lib
#OPEN_CV           = `pkg-config --cflags --libs opencv`

##### Combine variables
INCLUDES += $(BASE_INCLUDE)

EXECUTABLES = $(patsubst src/%.cpp, bin/%, $(wildcard src/*.cpp))

default: picopter

picopter: picopter.o waypoints.o
	$(CC) $(LDFLAGS) $(INCLUDES) $^ webInterface.o picopter_types.o $(BASE_OBJECTS) -o $@ $(LIBS) $(LDFLAGS)
	
picopter.o: picopter.cpp
	thrift --gen cpp picopter.thrift
	thrift --gen php picopter.thrift
	$(CC) $(CFLAGS) $(THRIFT_FLAGS) $(THRIFT_INC) $(INCLUDES) $^ $(THRIFT_SRC) $(LIBS) $(LDFLAGS)
	
waypoints.o: waypoints.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $^
	
clean:
	rm -f picopter.o waypoints.o picopter

#all: $(EXECUTABLES)
#
#bin/% : obj/%.o $(BASE_OBJECTS)
#	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_OBJECTS) $(RASPICAM_INCLUDES) $(RASPICAM_FLAGS) $(OPEN_CV) $< -o $@
#
#
#obj/%.o :  src/%.cpp $(BASE_HEADERS)
#	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) $< -o $@
#
#
#clean:
#	rm -f obj/*.o bin/* 
