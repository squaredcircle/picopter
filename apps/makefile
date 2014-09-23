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

XSENS_LINKS = -L ../../lib/Xsens -lcmt -lpthread -lrt
XSENS_INCLUDES = -I../../lib/Xsens
XSENS_OBJECTS = $(addprefix ../../lib/Xsens/, cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o)

RASPICAM_LINKS = -L../../lib/robidouille/raspicam_cv -L../../lib/raspberrypi/userland/build/lib -lraspicamcv -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host
RASPICAM_INCLUDES = -I../../lib/robidouille/raspicam_cv
OPEN_CV = `pkg-config --cflags --libs opencv`

EXECUTABLES = $(patsubst src/%.cpp, bin/%, $(wildcard src/*.cpp))


all: $(EXECUTABLES)

bin/% : obj/%.o $(BASE_OBJECTS) $(MODULES_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(MODULES_INCLUDE) $(MODULES_OBJECTS) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

obj/%.o :  src/%.cpp $(BASE_HEADERS) $(MODULES_HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(MODULES_INCLUDE) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) $< -o $@


clean:
	rm -f obj/*.o bin/* 
