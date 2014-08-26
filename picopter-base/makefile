CC = g++

CFLAGS = -Wall -c -g
INCLUDES = -I./include -I/usr/local/include -L/usr/local/lib
LDFLAGS = -lwiringPi -lboost_thread -lboost_system -lncurses -g

BASE = gpio logger flightBoard gps_qstarz imu_euler camera
BASE_SOURCES = $(addprefix src/, $(addsuffix .cpp, $(BASE)))
BASE_HEADERS = $(addprefix include/, $(addsuffix .h, $(BASE)))
BASE_OBJECTS = $(addprefix obj/, $(addsuffix .o, $(BASE)))

EXAMPLE = flightBoard_test gps_qstarz_test imu_euler_test camera_test
EXAMPLE_SOURCES = $(addprefix src/, $(addsuffix .cpp, $(EXAMPLE)))
EXECUTABLES = $(addprefix bin/, $(EXAMPLE))


XSENS_FLAGS =
XSENS_INCLUDES = -I../Xsens
XSENS_OBJECTS = $(addprefix ../Xsens/, cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o xsens_exception.o)

RASPICAM_FLAGS = -lraspicamcv -lmmal_core -lmmal -lmmal_util -lvcos -lbcm_host
RASPICAM_INCLUDES = -I../raspicam_cv -L../raspicam_cv -L../userland/build/lib
OPEN_CV = `pkg-config --cflags --libs opencv`


all: $(EXECUTABLES)


obj/gpio.o : include/gpio.h src/gpio.cpp
	$(CC) $(CFLAGS) $(INCLUDES) src/gpio.cpp -o obj/gpio.o

obj/logger.o : include/logger.h src/logger.cpp
	$(CC) $(CFLAGS) $(INCLUDES) src/logger.cpp -o obj/logger.o

obj/flightBoard.o : include/gpio.h src/gpio.cpp include/logger.h src/logger.cpp include/flightBoard.h src/flightBoard.cpp
	$(CC) $(CFLAGS) $(INCLUDES) src/flightBoard.cpp -o obj/flightBoard.o

obj/gps_qstarz.o : include/gpio.h src/gpio.cpp include/logger.h src/logger.cpp include/gps_qstarz.h src/gps_qstarz.cpp
	$(CC) $(CFLAGS) $(INCLUDES) src/gps_qstarz.cpp -o obj/gps_qstarz.o

obj/imu_euler.o : include/gpio.h src/gpio.cpp include/logger.h src/logger.cpp include/imu_euler.h src/imu_euler.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(XSENS_INCLUDES) src/imu_euler.cpp -o obj/imu_euler.o

obj/camera.o : include/gpio.h src/gpio.cpp include/logger.h src/logger.cpp include/camera.h src/camera.cpp
	$(CC) $(CFLAGS) $(INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) src/camera.cpp -o obj/camera.o





bin/flightBoard_test : obj/flightBoard_test.o $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_OBJECTS) $(RASPICAM_INCLUDES) $(RASPICAM_FLAGS) $(OPEN_CV) obj/flightBoard_test.o -o bin/flightBoard_test

bin/gps_qstarz_test : obj/gps_qstarz_test.o $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_OBJECTS) $(RASPICAM_INCLUDES) $(RASPICAM_FLAGS) $(OPEN_CV) obj/gps_qstarz_test.o -o bin/gps_qstarz_test
	
bin/imu_euler_test : obj/imu_euler_test.o $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_OBJECTS) $(RASPICAM_INCLUDES) $(RASPICAM_FLAGS) $(OPEN_CV) obj/imu_euler_test.o -o bin/imu_euler_test

bin/camera_test : obj/camera_test.o $(BASE_OBJECTS)
	$(CC) $(LDFLAGS) $(INCLUDES) $(BASE_OBJECTS) $(XSENS_INCLUDES) $(XSENS_OBJECTS) $(RASPICAM_INCLUDES) $(RASPICAM_FLAGS) $(OPEN_CV) obj/camera_test.o -o bin/camera_test



obj/flightBoard_test.o : src/flightBoard_test.cpp $(addprefix include/, $(addsuffix .h, flightBoard gpio logger))  $(addprefix src/, $(addsuffix .cpp, flightBoard gpio logger))
	$(CC) $(CFLAGS) $(INCLUDES) src/flightBoard_test.cpp -o obj/flightBoard_test.o

obj/gps_qstarz_test.o : src/gps_qstarz_test.cpp $(addprefix include/, $(addsuffix .h, gps_qstarz gpio logger)) $(addprefix src/, $(addsuffix .cpp, gps_qstarz gpio logger))
	$(CC) $(CFLAGS) $(INCLUDES) src/gps_qstarz_test.cpp -o obj/gps_qstarz_test.o

obj/imu_euler_test.o : src/imu_euler_test.cpp $(addprefix include/, $(addsuffix .h, imu_euler gpio logger)) $(addprefix src/, $(addsuffix .cpp, imu_euler gpio logger))
	$(CC) $(CFLAGS) $(INCLUDES) $(XSENS_INCLUDES) src/imu_euler_test.cpp -o obj/imu_euler_test.o

obj/camera_test.o : src/camera_test.cpp $(addprefix include/, $(addsuffix .h, camera gpio logger)) $(addprefix src/, $(addsuffix .cpp, camera gpio logger))
	$(CC) $(CFLAGS) $(INCLUDES) $(RASPICAM_INCLUDES) $(OPEN_CV) src/camera_test.cpp -o obj/camera_test.o


clean:
	rm -f obj/*.o bin/* 
