CC = g++

CFLAGS = -Wall -c -g
INCLUDES = -I./include -I/usr/local/include
LDFLAGS = -L/usr/local/lib -lwiringPi -lboost_thread -lboost_system -lncurses -g

BASE_INCLUDE = -I../base/include
BASE_HEADERS = $(wildcard ../base/include/*.h)
BASE_OBJECTS = $(wildcard ../base/obj/*.o)

XSENS_LINKS = -L ../../lib/Xsens -lcmt
XSENS_INCLUDES = -I../../lib/Xsens
XSENS_OBJECTS = $(addprefix ../../lib/Xsens/, cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o)

RASPICAM_LINKS = -L../../lib/robidouille/raspicam_cv -L../../lib/raspberrypi/userland/build/lib -lraspicamcv -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host
RASPICAM_INCLUDES = -I../../lib/robidouille/raspicam_cv
OPEN_CV = `pkg-config --cflags --libs opencv`

OBJECTS = $(patsubst src/%.cpp, obj/%.o, $(wildcard src/*.cpp))


all: $(OBJECTS)

obj/%.o :  src/%.cpp include/%.h $(BASE_HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) $(BASE_INCLUDE) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) $< -o $@


clean:
	rm -f obj/*.o 
