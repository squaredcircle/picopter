CC = g++

CFLAGS = -Wall -c -g
INCLUDES = -I./include -I/usr/local/include
LDFLAGS = -L/usr/local/lib -lwiringPi -lboost_thread -lboost_system -lncurses -lpthread -g

BASE_INCLUDE = -I../base/include
BASE_HEADERS = $(wildcard ../base/include/*.h)
BASE_OBJECTS = $(wildcard ../base/obj/*.o)

MODULES_INCLUDE = -I../modules/include
MODULES_HEADERS = $(wildcard ../modules/include/*.h)
MODULES_OBJECTS = $(wildcard ../modules/obj/*.o) 

XSENS_DIR = ../../lib/Xsens
XSENS_LINKS = -L$(XSENS_DIR) -lcmt -lpthread -lrt
XSENS_INCLUDES = -I$(XSENS_DIR)
XSENS_OBJECTS = $(addprefix $(XSENS_DIR)/, cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o)

RASPICAM_DIR = ../../lib/robidouille/raspicam_cv
USERLAND_DIR = ../../lib/userland
RASPICAM_LINKS = -L$(RASPICAM_DIR) -L$(USERLAND_DIR)/build/lib -lraspicamcv -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host
RASPICAM_INCLUDES = -I$(RASPICAM_DIR)
OPEN_CV = `pkg-config --cflags --libs opencv`

EXECUTABLES = $(patsubst src/%.cpp, bin/%, $(wildcard src/*.cpp))


all: $(EXECUTABLES)

bin/% : obj/%.o $(BASE_OBJECTS) $(MODULES_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(MODULES_INCLUDE) $(filter-out ../modules/obj/waypoints_loop%.o ../modules/obj/run_lawnmower.o ../modules/obj/lawnmower%.o, $(MODULES_OBJECTS)) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

bin/distance : obj/distance.o $(BASE_OBJECTS) $(MODULES_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(MODULES_INCLUDE) $(filter-out ../modules/obj/waypoints_loop%.o ../modules/obj/run_lawnmower.o, $(MODULES_OBJECTS)) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

bin/lawnmower : obj/lawnmower.o $(BASE_OBJECTS) $(MODULES_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(MODULES_INCLUDE) $(filter-out ../modules/obj/waypoints_loop%.o, $(MODULES_OBJECTS)) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@


#don't ask me.  ask gnu make.
bin/run_waypoints1 : obj/run_waypoints1.o $(BASE_OBJECTS) $(MODULES_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(MODULES_INCLUDE) $(MODULES_OBJECTS) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

bin/run_waypoints2 : obj/run_waypoints2.o $(BASE_OBJECTS) $(MODULES_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(MODULES_INCLUDE) $(MODULES_OBJECTS) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

bin/run_waypoints3 : obj/run_waypoints3.o $(BASE_OBJECTS) $(MODULES_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(MODULES_INCLUDE) $(MODULES_OBJECTS) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

bin/run_waypoints4 : obj/run_waypoints4.o $(BASE_OBJECTS) $(MODULES_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(MODULES_INCLUDE) $(MODULES_OBJECTS) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

bin/run_waypoints5 : obj/run_waypoints5.o $(BASE_OBJECTS) $(MODULES_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(MODULES_INCLUDE) $(MODULES_OBJECTS) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@


obj/%.o :  src/%.cpp $(BASE_HEADERS) $(MODULES_HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(MODULES_INCLUDE) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) $< -o $@


clean:
	rm -f obj/*.o bin/* 
