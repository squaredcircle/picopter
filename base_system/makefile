CC = g++

CFLAGS = -I. -I/usr/local/include -L/usr/local/lib -Wall -Werror
LDFLAGS = -lwiringPi -lboost_thread
SOURCES = gpio.cpp logger.cpp flightBoard.cpp gps_qstarz.cpp square.cpp
OBJECTS = $(SOURCES: .cpp = .o)
EXECUTABLE = square

all:	$(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@


clean:
		rm -f *.o
		rm -f $(EXECUTABLE)

