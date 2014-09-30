##### General
CC                = g++
CFLAGS            = -O2 -Wall -c
INCLUDES          = -I/usr/local/include  -Iinclude
LIBS              = -L/usr/local/lib -L/usr/lib -L../../lib/Xsens
LDFLAGS           = -lwiringPi -lthrift -lboost_thread -lboost_system -lncurses -lpthread -lcmt -lrt

##### Thrift
THRIFT_FLAGS      = -DHAVE_INTTYPES_H -DHAVE_NETINET_IN_H
THRIFT_INCLUDES   = -Igen-cpp -I/usr/local/include/thrift

##### picopter-base
BASE              = gpio logger flightBoard config_parser gps_qstarz imu_euler 
BASE_INCLUDES     = -I../base/include
BASE_OBJECTS      = $(addprefix ../base/obj/, $(addsuffix .o, $(BASE)))

##### picopter-modules
MODULES              = run_lawnmower lawnmower_control lawnmower_structures detectObjects
MODULES_INCLUDES     = -I../modules/include
MODULES_OBJECTS      = $(addprefix ../modules/obj/, $(addsuffix .o, $(MODULES)))

##### IMU
XSENS_INCLUDES    = -I../../lib/Xsens
XSENS_OBJECTS     = $(addprefix ../../lib/Xsens/, cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o)

##### Vision
RASPICAM_LINKS    = -L../../lib/robidouille/raspicam_cv -L../../lib/raspberrypi/userland/build/lib -lraspicamcv -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host
RASPICAM_INCLUDES = -I../../lib/robidouille/raspicam_cv
OPEN_CV           = `pkg-config --cflags --libs opencv`
	
default: gen-cpp/picopter_types.h bin/picopter
	
gen-cpp/picopter_types.h: picopter.thrift
	thrift --gen cpp picopter.thrift
	thrift --gen php picopter.thrift
	
bin/picopter: obj/webInterface.o obj/picopter_types.o obj/picopter.o obj/waypoints.o obj/control.o obj/structures.o
	$(CC) $(INCLUDES) $^ $(BASE_OBJECTS) $(MODULES_OBJECTS) -o $@ $(LIBS) $(RASPICAM_LINKS) $(OPEN_CV) $(LDFLAGS)
	
obj/webInterface.o: gen-cpp/webInterface.cpp
	$(CC) $(CFLAGS) $(THRIFT_FLAGS) $(INCLUDES) $(THRIFT_INCLUDES) $^ -o $@ $(LIBS)
	
obj/picopter_types.o: gen-cpp/picopter_types.cpp
	$(CC) $(CFLAGS) $(THRIFT_FLAGS) $(INCLUDES) $(THRIFT_INCLUDES) $^ -o $@ $(LIBS)
	
obj/picopter.o: src/picopter.cpp
	$(CC) $(CFLAGS) $(THRIFT_FLAGS) $(INCLUDES) $(BASE_INCLUDES) $(MODULES_INCLUDES) $(XSENS_INCLUDES) $(THRIFT_INCLUDES) $^ -o $@ $(LIBS)
	
obj/waypoints.o: src/waypoints.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDES) $(XSENS_INCLUDES) $^ -o $@
	
obj/control.o: src/control.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDES) $(XSENS_INCLUDES) $^ -o $@

obj/structures.o: src/structures.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDES) $(XSENS_INCLUDES) $^ -o $@
	
clean:
	rm -f obj/*.o bin/*
