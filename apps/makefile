CC = g++

CFLAGS = -Wall -c -g
INCLUDES = -I./include -I/usr/local/include
LDFLAGS = -L/usr/local/lib -lwiringPi -lboost_thread -lboost_system -lncurses -g

BASE_INCLUDE = -I../base/include
BASE_HEADERS = $(wildcard ../base/include/*.h)
BASE_OBJECTS = $(wildcard ../base/obj/*.o)

XSENS_LINKS = -L ../Xsens -lcmt
XSENS_INCLUDES = -I../Xsens
XSENS_OBJECTS = $(addprefix ../Xsens/, cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o)

RASPICAM_LINKS = -L../../robidouille/raspicam_cv -L../../userland/build/lib -lraspicamcv -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host
RASPICAM_INCLUDES = -I../../robidouille/raspicam_cv
OPEN_CV = `pkg-config --cflags --libs opencv`

EXECUTABLES = $(patsubst src/%.cpp, bin/%, $(wildcard src/*.cpp))


all: $(EXECUTABLES)

bin/% : obj/%.o $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

obj/%.o :  src/%.cpp $(BASE_HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) $< -o $@


clean:
	rm -f obj/*.o bin/* 
