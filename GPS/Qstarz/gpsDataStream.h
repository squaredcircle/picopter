#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <string>

#include "errno.h"
#include "wiringSerial.h"
#include "wiringPi.h"

std::string gpsDataStream(int);
