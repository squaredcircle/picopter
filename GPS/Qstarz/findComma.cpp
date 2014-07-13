#include "findComma.h"

int found;
int pos;
int count;
int commaPos;
char gpsChar;

int findComma(std::string data, int commaNo) {
	int found = 0;
	int pos = 0;
	int count = 0;
	int commaPos = 0;
	while(!found) {
		gpsChar = data[pos];
		if (gpsChar == ',') {
			count++;
			if (count == commaNo) {
				commaPos = pos;
				//std::cout << "Position at: " << commaPos << "\n";
				found = 1;
			}
		}
		pos++;
	}
	return commaPos;
}
