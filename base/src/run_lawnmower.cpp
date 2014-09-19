//Basic function that causes the Hexacpter to search a square, lawnmower fashion
//Written by Omid Targhagh, based on work done by Michael Baxter

#include "run_lawnmower.h"
bool exitLawnmower = false;
bool usingIMU = true;

void run_lawnmower(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, IMU *imuPtr, IMU_Data *compDataPtr, Pos start, Pos end) {

	cout << "Starting to run lawnmower..." << endl;
	if(imuPtr->setup() != IMU_OK) {		//Check if IMU
        cout << "Error opening imu: Will navigate using GPS only." << endl;
        usingIMU = false;
    }
	//Start the camera up & load image of James Oval
	RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0);
	Mat oval = imread(OVAL_IMAGE_PATH);
	if (oval.empty()) {	//Checks for loading errors
		cout << "Error loading the image file " << OVAL_IMAGE_PATH << " Terminating program." << endl;
		return;
	}

	Logger lawnlog = Logger("Lawn.log");	//Initalise logs
	Logger rawgpslog = Logger("Lawn_Raw_GPS.txt");
	char str[BUFSIZ];

	vector<Pos> gpsPoints;
	populateMainVector(&gpsPoints, &lawnlog, start, end);
	
	cout << endl;
	for(int i = 0; i < (int)gpsPoints.size(); i++) {
		cout << setprecision(15) << "Point " << i+1 << " is " << (gpsPoints[i].lat) << " " << (gpsPoints[i].lon) << endl;
		sprintf(str, "Point %d is %f %f", i+1, (gpsPoints[i].lat), (gpsPoints[i].lon));
		lawnlog.writeLogLine(str);
	}
	cout << endl;

	cout  << "Waiting to enter autonomous mode..." << endl;
	while(!gpio::isAutoMode()) delay(100);	//Hexacopter waits until put into auto mode
	cout << "Autonomous Mode has been Entered" << endl;

	double yaw;
	if (usingIMU) {
		imuPtr->getIMU_Data(compDataPtr);
		yaw = compDataPtr->yaw;
		cout << "Using compass: Copter is facing " << yaw << " degrees." << endl;
		sprintf(str, "Using compass: Copter is facing %f degrees.", yaw);
	}
	else {
		yaw = determineBearing(fbPtr, gpsPtr, dataPtr);	//Hexacopter determines which way it is facing
		sprintf(str, "Bearing found with GPS: Copter is facing %f degrees.", yaw);
	}
	lawnlog.writeLogLine(str);
	gpsPtr->getGPS_Data(dataPtr);		//Hexacopter works out where it is
	cout << "Location and Orienation determined" << endl;
	
	for (int i = 0; i < (int)gpsPoints.size(); i++) {
		flyTo(fbPtr, gpsPtr, dataPtr, imuPtr, compDataPtr, gpsPoints[i].lat, gpsPoints[i].lon, yaw, &lawnlog, &rawgpslog, capture, i, oval);
		if (i == 0) {	//Are we at the first point?
			rawgpslog.clearLog();			//Flush data in there - also removers header
			oval = imread(OVAL_IMAGE_PATH);	//Wipe any extra lines caused by flying to first point
		}
		if(exitLawnmower) {
			break;
		}
	}

	sprintf(str, "photos/James_Oval_%d.jpg", (int)((dataPtr->time)*100));
	imwrite(str, oval);
	raspiCamCvReleaseCapture(&capture);
	cout << "Done!" << endl;
	lawnlog.writeLogLine("Finished!");
}

void flyTo(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, IMU *imuPtr, IMU_Data *compDataPtr, double targetLat, double targetLon, double yaw, Logger *logPtr, Logger *rawLogPtr, RaspiCamCvCapture *camPtr, int index, Mat oval) {
	FB_Data stop = {0, 0, 0, 0};
	FB_Data course = {0, 0, 0, 0};
	Pos start, end;

	gpsPtr->getGPS_Data(dataPtr);
	if (usingIMU){
		imuPtr->getIMU_Data(compDataPtr);
		yaw = compDataPtr->yaw;
	}
	imshow("Oval Map", oval);	//Constantly updates picture
	waitKey(1);
	start.lat = (dataPtr->latitude);
	start.lon = (dataPtr->longitude);
	end.lat = targetLat;
	end.lon = targetLon;
	cout << setprecision(15) << "Flying to " << targetLat << " " << targetLon << ", facing " << yaw << " degrees" << endl;
	char str[BUFSIZ];
	sprintf(str, "%f %f %d", dataPtr->longitude, dataPtr->latitude, index);
	rawLogPtr->writeLogLine(str, false);
	double distance = calculate_distance(start, end);
	double bearing = calculate_bearing(start, end);
	cout << "Distance: " << distance << " m\tBearing: " << bearing << " degrees" << endl;

	Mat bestImg;
	Mat currentImg;
	int timer = 0;
	bool sawRed = false;
	bool haveBest = false;

	while (!exitLawnmower && distance > WAYPOINT_RADIUS) {
		setLawnCourse(&course, distance, bearing, yaw);
		sprintf(str, "Course set to : {%d (A), %d (E)}", course.aileron, course.elevator);
		logPtr->writeLogLine(str);
		fbPtr->setFB_Data(&course);
		IplImage* view = raspiCamCvQueryFrame(camPtr);
		Mat imBGR(view);
		Mat image;
		cvtColor(imBGR, image, CV_BGR2HSV);
		timer++;
		if ((timer > 0) && checkRed(image, logPtr)) {	//Is there red?
			sawRed = true;
			image.copyTo(currentImg);
			if (!haveBest) {
				currentImg.copyTo(bestImg);
				haveBest = true;
			}
			else if (redComDist(currentImg) < redComDist(bestImg)) {
				currentImg.copyTo(bestImg);
			}
		}
		else {
			if (sawRed && (timer > 0))  {	//Only resets first time image leaves frame
				timer = 0-FRAME_WAIT;
				sprintf(str, "photos/Lawnmower_%d_%d_%d.jpg", (int)((dataPtr->latitude)*1000), (int)((dataPtr->longitude)*1000), (int)((dataPtr->time)*100));
				imwrite(str, bestImg);
				//imshow("Last Red Object", bestImg);
				waitKey(1);
				haveBest = false;
			}
			sawRed = false;
		}
		
		delay(LOOP_WAIT);	//Wait for instructions
		gpsPtr->getGPS_Data(dataPtr);
		if (usingIMU) {
			imuPtr->getIMU_Data(compDataPtr);
			yaw = compDataPtr->yaw;
		}
		updatePicture(oval, dataPtr->latitude, dataPtr->longitude);	
		namedWindow("Oval Map", CV_WINDOW_AUTOSIZE);
		imshow("Oval Map", oval);
		waitKey(1); 
		start.lat = (dataPtr->latitude);
		start.lon = (dataPtr->longitude);
		cout << "Needs to move from: " << dataPtr->latitude << ", " << dataPtr->longitude << "\n\tto : " << targetLat << ", " << targetLon << endl;
		sprintf(str, "Currently at %f %f", dataPtr->latitude, dataPtr->longitude);
		logPtr->writeLogLine(str);
		sprintf(str, "%f %f %d", dataPtr->longitude, dataPtr->latitude, index);
		rawLogPtr->writeLogLine(str, false);
		sprintf(str, "Going to %f %f", end.lat, end.lon);
		logPtr->writeLogLine(str);
		distance = calculate_distance(start, end);
		bearing = calculate_bearing(start, end);
		cout << "Distance: " << distance << " m\tBearing: " << bearing << endl;
		sprintf(str, "Distance: %f m\tBearing : %f degrees", distance, bearing);
		logPtr->writeLogLine(str);
		if (!gpio::isAutoMode()) {
			terminateLawn(0);
			return;
		}
	}
	cout << "Arrived" << endl;
	sprintf(str, "Arrived at %f %f\n----------------------------------\n", end.lat, end.lon);
	logPtr->writeLogLine(str);
	fbPtr->setFB_Data(&stop);
	delay(LOCATION_WAIT);
}

bool checkRed(Mat image, Logger *logPtr) {
	int nRows = image.rows;
	int nCols = image.cols;
	uchar* p;
	int nRed = 0;
	for(int i = 0; i < nRows; i++) {
		p = image.ptr<uchar>(i);
		for (int j = 0; j < nCols; j=j+3) {
			if (((p[j] > HMIN) || (p[j] < HMAX)) && (p[j] > SMIN) && (p[j] < SMAX) && (p[j] > VALMIN) && (p[j] < VALMAX)) {
				nRed++;
			}
		}
	}
	cout << "How much 'Red' can we see? " << nRed << endl;
	char str[BUFSIZ];
	sprintf(str, "We can see %d 'Red' pixels.", nRed);
	logPtr->writeLogLine(str);
	if (nRed >= REDTHRESH) return true;
	else return false;
}

double redComDist(Mat image) {
	int nRows = image.rows;
	int nCols = image.cols;
	uchar* p;
	double xMean = 0;
	double yMean = 0;
	int nRed = 0;
	for(int i = 0; i < nRows; i++) {
		p = image.ptr<uchar>(i);
		for (int j = 0; j < nCols; j=j+3) {
			if (((p[j] > HMIN) || (p[j] < HMAX)) && (p[j] > SMIN) && (p[j] < SMAX) && (p[j] > VALMIN) && (p[j] < VALMAX)) {
				nRed++;
				xMean = xMean + j/3;
				yMean = yMean + i;
			}
		}
	}
	xMean = xMean/nRed;
	yMean = yMean/nRed;
	return sqrt(pow(xMean-(double)(nCols/2), 2) + pow(yMean-(double)(nRows/2), 2));	//Mean distance
}

void updatePicture(Mat oval, double latitude, double longitude) {
	if ((latitude < MINLAT) || (latitude > MAXLAT) || (longitude < MINLON) || (longitude > MAXLON)) return; //Are we inside the image?
	int row = (oval.rows)*(latitude - MAXLAT)/(MINLAT - MAXLAT);
	int column = (oval.cols)*(longitude - MINLON)/(MAXLON - MINLON);
	if ((row - PIXEL_RADIUS) < 0) row = PIXEL_RADIUS;					//Check if we are going out of bnounds of the image
	if ((row + PIXEL_RADIUS) > oval.rows) row = oval.rows - PIXEL_RADIUS;
	if ((column - PIXEL_RADIUS) < 0) column = PIXEL_RADIUS;
	if ((column + PIXEL_RADIUS) > oval.cols) column = oval.cols - PIXEL_RADIUS;
	for (int i = row; i <= row + PIXEL_RADIUS; i++) {
		for (int j = column - PIXEL_RADIUS; j <= column + PIXEL_RADIUS; j++){
			uchar *pixelPtr = oval.ptr<uchar>(i, j);
			pixelPtr[0] = 0;	//Draw a black line
			pixelPtr[1] = 0;
			pixelPtr[2] = 0;
		}
	}
}

//Old bearing function - GPS only -----------------------------------------
double determineBearing(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr) {
	cout << "The Hexacopter wil now determine it's orientation." << endl;
	FB_Data stop = {0, 0, 0, 0};							//Predefine FB commands
	FB_Data forwards = {0, DIRECTION_TEST_SPEED, 0, 0};
	Pos test_start;											//To work out initial heading, we calculate the bearing
	Pos test_end;											//form the start coord to the end coord.

	gpsPtr->getGPS_Data(dataPtr);													//Record start position.		
	test_start.lat = (dataPtr->latitude);
	test_start.lon = (dataPtr->longitude);
	fbPtr->setFB_Data(&forwards);										//Tell flight board to go forwards.
	delay(DIRECTION_TEST_DURATION);										//Wait a bit (travel).
	fbPtr->setFB_Data(&stop);											//Stop.
	gpsPtr->getGPS_Data(dataPtr);										//Record end position.
	test_end.lat = (dataPtr->latitude);
	test_end.lon = (dataPtr->longitude);

	double yaw = calculate_bearing(test_start, test_end);	//Work out which direction we went.
	cout << "The Hexacopter has an orientation of: " << yaw << endl;
	return yaw;
}

void setLawnCourse(FB_Data *instruction, double distance, double bearing, double yaw) {
	double speed = KPh * distance;
	if(speed > SPEED_LIMIT) {											//P controler with limits.
		speed = SPEED_LIMIT;
	}
	instruction->aileron = (int) (speed * sin((bearing-yaw)*(PI/180)));
	instruction->elevator = (int) (speed * cos((bearing-yaw)*(PI/180)));
	instruction->rudder = 0;
	instruction->gimbal = 0;
}

double calculate_distance(Pos pos1, Pos pos2) {
	double lat1 = (pos1.lat)*(PI/180);	//Convert into radians
	double lon1 = (pos1.lon)*(PI/180);
	double lat2 = (pos2.lat)*(PI/180);
	double lon2 = (pos2.lon)*(PI/180);
	double h = sin2((lat1-lat2)/2) + cos(lat1)*cos(lat2) * sin2((lon2-lon1)/2);
	if(h > 1) cout << "Distance calculation error" << endl;
	double distance = 2 * RADIUS_OF_EARTH * asin(sqrt(h));
	return distance;	//meters
}

double calculate_bearing(Pos pos1, Pos pos2) {
	double lat1 = (pos1.lat)*(PI/180);	//Convert into radians
	double lon1 = (pos1.lon)*(PI/180);
	double lat2 = (pos2.lat)*(PI/180);
	double lon2 = (pos2.lon)*(PI/180);
	double num = sin(lon2 - lon1) * cos(lat2);
	double den = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2 - lon1);
	double bearing = atan2(num, den);
	return bearing*(180/PI);	//In degrees
}

void populateMainVector(vector<Pos> *list, Logger *logPtr, Pos start, Pos end) {
	char str[BUFSIZ];
	Pos corners[4];
	corners[0] = start;
	cout << "Corner #1 read as: " << (corners[0].lat) << " " << (corners[0].lon) << endl;
	sprintf(str, "Corner #1 read as: %f %f", (corners[0].lat), (corners[0].lon));
	logPtr->writeLogLine(str);	
	corners[3] = end;
	cout << "Corner #4 read as: " << (corners[3].lat) << " " << (corners[3].lon) << endl;
	sprintf(str, "Corner #4 read as: %f %f", (corners[3].lat), (corners[3].lon));
	logPtr->writeLogLine(str);	
	corners[1].lat = corners[0].lat;
	corners[1].lon = corners[3].lon;
	cout << "Corner #2 calculated as: " << (corners[1].lat) << " " << (corners[1].lon) << endl;
	sprintf(str, "Corner #2 calculated as: %f %f", (corners[1].lat), (corners[1].lon));
	logPtr->writeLogLine(str);	
	corners[2].lat = corners[3].lat;
	corners[2].lon = corners[0].lon;
	cout << "Corner #3 calculated as: " << (corners[2].lat) << " " << (corners[2].lon) << endl;
	sprintf(str, "Corner #3 calculated as: %f %f", (corners[2].lat), (corners[2].lon));
	logPtr->writeLogLine(str);

	double lonDistance = calculate_distance(corners[0], corners[2]);	//Find separation of 'top' and 'bottom' of sweep
	double otherDist = calculate_distance(corners[1], corners[3]);
	if (otherDist < lonDistance) lonDistance = otherDist;
	int lonPoints = (int)(lonDistance/SPACING) + 1;							//Number of points along each sweep
	int direction = 1;
	if (corners[0].lon > corners[3].lon) {
		direction = -1;	//Is corners[0] East of corners[3] instead of West?
	}
	vector<Pos> topSide, bottomSide;	//Ends of sweeps
	double fraction, distance, angle;
	Pos dummyPos;
	for (int i = 0; i < lonPoints; i++) {
		fraction = (double)i/lonPoints;
		distance = fraction/lonDistance;
		angle = distance/(RADIUS_OF_EARTH*cos((corners[0].lat)*(PI/180)))*(180/PI);	//'Top' side on same latitude as corners[0]
		dummyPos.lat = corners[0].lat;
		dummyPos.lon = corners[0].lon + (double)direction*angle;
		topSide.push_back(dummyPos);
		cout << "Point " << dummyPos.lat << " " << dummyPos.lon << " added." << endl;
		angle = distance/(RADIUS_OF_EARTH*cos((corners[2].lat)*(PI/180)))*(180/PI);	//'Bottom' side on same latitude as corners[2]
		dummyPos.lat = corners[2].lat;
		dummyPos.lon = corners[2].lon + (double)direction*angle;
		bottomSide.push_back(dummyPos);
		cout << "Point " << dummyPos.lat << " " << dummyPos.lon << " added." << endl;
	}
	int sweeps = topSide.size();
	cout << "Sides calculated... Each side contains " << sweeps+1 << " points." << endl;
	list->push_back(corners[0]);	//Start of lawnmower pattern
	for (int i = 0; i < sweeps - 1; i++) {
		cout <<i<<endl;
		if (i % 2 == 0) {	//Going from 'top' to 'bottom', then along 'bottom'
			cout << "Even" << endl;
			addPoints(list, topSide[i], bottomSide[i], 1);
			addPoints(list, bottomSide[i], bottomSide[i+1], 2);
		}
		else {	//Other way
			cout << "Odd" << endl;
			addPoints(list, bottomSide[i], topSide[i], 1);
			addPoints(list, topSide[i], topSide[i+1], 2);
		}
	}
	if (sweeps % 2 == 1) {	//Need to add in last 'top' to 'bottom' sweep
		addPoints(list, topSide[sweeps-1], bottomSide[sweeps-1], 1);
	}
	else {	//... Or 'bottom' to 'top'
		addPoints(list, bottomSide[sweeps-1], topSide[sweeps-1], 1);
	}
	cout << "All waypoints calulated." << endl;
}

void addPoints(vector<Pos> *list, Pos start, Pos end, int way) {
	//cout << "Adding points..." << endl;
	double endDistance = calculate_distance(start, end);
	int points = (int)(endDistance/SPACING) + 1;	//Number of intermediate points
	double fraction, distance, angle;
	Pos dummyPos;
	int direction = 1;
	if (way == 1) {	//Starting and ending on same longitude
		if (start.lat < end.lat) {
			direction = -1;	//Are we going 'up' instaed?
		}
		for (int i = 1; i < points; i++) {	//Start point already added
			fraction = (double)i/points;
			distance = fraction/endDistance;
			angle = distance/RADIUS_OF_EARTH*(180/PI);	//'Top' side on same latitude as corners[0]
			dummyPos.lat = start.lat + (double)direction*angle;
			dummyPos.lon = start.lon;
			list->push_back(dummyPos);
			//cout << "Point " << dummyPos.lat << " " << dummyPos.lon << " added." << endl;
		}
	}
	else if (way == 2) {	//Starting and ending on same latitude
		if (start.lon > end.lon) {
			direction = -1;	//Are we going 'left' instead?
		}
		for (int i = 1; i < points; i++) {	//Start point already added
			fraction = (double)i/points;
			distance = fraction/endDistance;
			angle = distance/(RADIUS_OF_EARTH*cos((start.lat)*(PI/180)))*(180/PI);	//'Top' side on same latitude as corners[0]
			dummyPos.lat = start.lat;
			dummyPos.lon = start.lon + (double)direction*angle;
			list->push_back(dummyPos);
			cout << "Point " << dummyPos.lat << " " << dummyPos.lon << " added." << endl;
		}
	}
	list->push_back(end);	//Add end point
}

void terminateLawn(int signum) {
	cout << "Signal " << signum << " received. Quitting lawnmower program." << endl;
	exitLawnmower = true;
}
