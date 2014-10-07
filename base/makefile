CC = g++

CFLAGS = -Wall -c -g
INCLUDES = -I./include -I/usr/local/include
LDFLAGS = -L/usr/local/lib -lwiringPi -lboost_thread -lboost_system -lpthread -lncurses -g

CORE = gpio logger config_parser
CORE_SOURCES = $(addprefix src/, $(addsuffix .cpp, $(CORE)))
CORE_HEADERS = $(addprefix include/, $(addsuffix .h, $(CORE)))
CORE_OBJECTS = $(addprefix obj/, $(addsuffix .o, $(CORE)))

BASE = $(patsubst src/%.cpp, %, $(filter-out src/%test.cpp $(CORE_SOURCES), $(wildcard src/*.cpp)))
BASE_SOURCES = $(addprefix src/, $(addsuffix .cpp, $(BASE)))
BASE_HEADERS = $(addprefix include/, $(addsuffix .h, $(BASE)))
BASE_OBJECTS = $(addprefix obj/, $(addsuffix .o, $(BASE)))

EXAMPLE = $(patsubst src/%.cpp, %, $(filter src/%test.cpp, $(wildcard src/*.cpp)))
EXAMPLE_SOURCES = $(addprefix src/, $(addsuffix .cpp, $(EXAMPLE)))
EXAMPLE_OBJECTS = $(addprefix obj/, $(addsuffix .o, $(EXAMPLE)))
EXECUTABLES = $(addprefix bin/, $(EXAMPLE))

XSENS_DIR = ../../lib/Xsens
XSENS_LINKS = -L$(XSENS_DIR) -lcmt -lpthread -lrt
XSENS_INCLUDES = -I$(XSENS_DIR)
XSENS_OBJECTS = $(addprefix $(XSENS_DIR)/, cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o)

RASPICAM_DIR = ../../lib/robidouille/raspicam_cv
USERLAND_DIR = ../../lib/userland
RASPICAM_LINKS = -L$(RASPICAM_DIR) -L$(USERLAND_DIR)/build/lib -lraspicamcv -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host
RASPICAM_INCLUDES = -I$(RASPICAM_DIR)
OPEN_CV = `pkg-config --cflags --libs opencv`


all: $(CORE_OBJECTS) $(BASE_OBJECTS) $(EXECUTABLES)


bin/% : obj/%.o $(CORE_OBJECTS) $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(CORE_OBJECTS) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

bin/%_test : obj/%_test.o $(CORE_OBJECTS) obj/%.o obj/camera.o
	$(CC) $(LDFLAGS) $(INCLUDES) $(CORE_OBJECTS) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_LINKS) $(RASPICAM_INCLUDES) $(RASPICAM_LINKS) $(OPEN_CV) $< -o $@

obj/%.o : src/%.cpp $(CORE_HEADERS) include/camera.h
	$(CC) $(CFLAGS) $(INCLUDES) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) $< -o $@

obj/%_test.o : src/%_test.cpp $(CORE_HEADERS) include/%.h include/camera.h
	$(CC) $(CFLAGS) $(INCLUDES) $(XSENS_INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) $< -o $@


clean:
	rm -f obj/*.o bin/* 
