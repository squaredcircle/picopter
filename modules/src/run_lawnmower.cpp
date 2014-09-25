//Basic function that causes the Hexacpter to search a square, lawnmower fashion
//Written by Omid Targhagh, based on work done by Michael Baxter
//Includes image detection of Merrick Cloete

#include "run_lawnmower.h"

bool exitLawnmower = false;
bool usingIMU = true;

void run_lawnmower(FlightBoard *fbPtr, GPS *gpsPtr, IMU *imuPtr, Pos start, Pos end) {

	cout << "Starting to run lawnmower..." << endl;
	
	ConfigParser::ParamMap lawnParameters;		//Load parameters from config file
    lawnParameters.insert("SPEED_LIMIT", &SPEED_LIMIT);
    lawnParameters.insert("SWEEP_SPACING", &SWEEP_SPACING);
    lawnParameters.insert("POINT_SPACING", &POINT_SPACING);
    lawnParameters.insert("WAYPOINT_RADIUS", &WAYPOINT_RADIUS);
    lawnParameters.insert("KPxy", &KPxy);
    lawnParameters.insert("KIxy", &KIxy);
    lawnParameters.insert("KPz", &KPz);
    lawnParameters.insert("KIz", &KIz);
    ConfigParser::loadParameters("LAWNMOWER", &lawnParameters, CONFIG_FILE);
    ConfigParser::ParamMap camParameters; 
    camParameters.insert("HMIN", &HMIN);
    camParameters.insert("HMAX", &HMAX);
    camParameters.insert("SMIN", &SMIN);
    camParameters.insert("SMAX", &SMAX);
    camParameters.insert("VMINIMUM", &VMINIMUM);
    camParameters.insert("VMAX", &VMAX);
    camParameters.insert("WHITE", &WHITE);
    camParameters.insert("BLACK", &BLACK);
    camParameters.insert("COLSIZE", &COLSIZE);
    camParameters.insert("ROWSIZE", &ROWSIZE);
    camParameters.insert("PIXELTHRESH", &PIXELTHRESH);
    camParameters.insert("DILATE_ELEMENT", &DILATE_ELEMENT);
    camParameters.insert("ERODE_ELEMENT", &ERODE_ELEMENT);
    ConfigParser::loadParameters("CAMERA", &camParameters, CONFIG_FILE);
	
	if(imuPtr->setup() != IMU_OK) {		//Check if IMU
        cout << "Error opening imu: Will navigate using GPS only." << endl;
        usingIMU = false;
    }
    IMU_Data compassdata;   
	GPS_Data data;
	//Start the camera up & load image of James Oval
	RaspiCamCvCapture* capture = raspiCamCvCreateCameraCapture(0);
	Mat oval = imread(OVAL_IMAGE_PATH);
	if (oval.empty()) {	//Checks for loading errors
		cout << "Error loading the image file " << OVAL_IMAGE_PATH << " Terminating program." << endl;
		return;
	}

	Logger lawnlog = Logger("Lawn.log");	//Initalise logs
	Logger rawgpslog = Logger("Lawn_Raw_GPS.txt");	//Easier to read into M/Matica
	char str[BUFSIZ];
	sprintf(str, "Config parameters set to:");	//Record parameters
	lawnlog.writeLogLine(str);
	sprintf(str, "\tSPEED_LIMIT\t%d", SPEED_LIMIT);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tSWEEP_SPACING\t%f", SWEEP_SPACING);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tPOINT_SPACING\t%f", POINT_SPACING);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tWAYPOINT_RADIUS\t%f", WAYPOINT_RADIUS);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tKPxy\t%f", KPxy);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tKIxy\t%f", KIxy);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tKPz\t%f", KPz);
	lawnlog.writeLogLine(str);
	sprintf(str, "\tKIz\t%f", KIz);
	lawnlog.writeLogLine(str);
	lawnlog.writeLogLine("\n");

	vector<Pos> gpsPoints;
	populateMainVector(&gpsPoints, &lawnlog, start, end);
	cout << endl;
	for(int i = 0; i < (int)gpsPoints.size(); i++) {
		cout << setprecision(15) << "Point " << i+1 << " is " << (gpsPoints[i].lat) << " " << (gpsPoints[i].lon) << endl;
		sprintf(str, "Point %d is %f %f", i+1, (gpsPoints[i].lat), (gpsPoints[i].lon));
		lawnlog.writeLogLine(str);
	}
	lawnlog.writeLogLine("\n");
	cout << endl;

	cout  << "Waiting to enter autonomous mode..." << endl;
	while(!gpio::isAutoMode()) delay(100);	//Hexacopter waits until put into auto mode
	cout << "Autonomous Mode has been Entered" << endl;

	double yaw;
	if (usingIMU) {
		imuPtr->getIMU_Data(&compassdata);
		yaw = compassdata.yaw;
		cout << "Using compass: Copter is facing " << yaw << " degrees." << endl;
		sprintf(str, "Using compass: Copter is facing %f degrees.", yaw);
	}
	else {
		yaw = determineBearing(fbPtr, gpsPtr, &data);	//Hexacopter determines which way it is facing
		sprintf(str, "Bearing found with GPS: Copter is facing %f degrees.", yaw);
	}
	lawnlog.writeLogLine(str);
	gpsPtr->getGPS_Data(&data);		//Hexacopter works out where it is
	cout << "Location and Orienation determined" << endl;
	
	for (int i = 0; i < (int)gpsPoints.size(); i++) {
		flyTo(fbPtr, gpsPtr, &data, imuPtr, &compassdata, gpsPoints[i], yaw, &lawnlog, &rawgpslog, capture, i, oval);
		if (i == 0) {	//Are we at the first point?
			rawgpslog.clearLog();			//Flush data in there - also removers header
			oval = imread(OVAL_IMAGE_PATH);	//Wipe any extra lines caused by flying to first point
		}
		if(exitLawnmower) {
			break;
		}
	}

	sprintf(str, "photos/James_Oval_%d.jpg", (int)((data.time)*100));
	imwrite(str, oval);
	raspiCamCvReleaseCapture(&capture);
	cout << "Done!" << endl;
	lawnlog.writeLogLine("Finished!");
}

void flyTo(FlightBoard *fbPtr, GPS *gpsPtr, GPS_Data *dataPtr, IMU *imuPtr, IMU_Data *compDataPtr, Pos end, double yaw, Logger *logPtr, Logger *rawLogPtr, RaspiCamCvCapture *camPtr, int index, Mat oval) {
	FB_Data stop = {0, 0, 0, 0};
	FB_Data course = {0, 0, 0, 0};
	Pos start;
	gpsPtr->getGPS_Data(dataPtr);
	if (usingIMU){
		imuPtr->getIMU_Data(compDataPtr);
		yaw = compDataPtr->yaw;
	}
	imshow("Oval Map", oval);	//Constantly updates picture
	waitKey(1);
	start.lat = (dataPtr->latitude);
	start.lon = (dataPtr->longitude);
	cout << setprecision(15) << "Flying to " << end.lat << " " << end.lon << ", facing " << yaw << " degrees" << endl;
	char str[BUFSIZ];
	sprintf(str, "%f %f %d", dataPtr->longitude, dataPtr->latitude, index);
	rawLogPtr->writeLogLine(str, false);
	double distance = calculate_distance(start, end);
	double bearing = calculate_bearing(start, end);
	cout << "Distance: " << distance << " m\tBearing: " << bearing << " degrees" << endl;

	double pastDistances[PAST_POINTS];		//Array of past distance values
	for (int i = 0; i < PAST_POINTS; i++) {
		pastDistances[i] = 0;
	}
	
	Mat bestImg, currentImg, imHSV, imBin, BGRTemp, image;
	IplImage* view;
	int nObjects = 0;
	int centres[OBJECT_LIMIT][2];
	int radii[OBJECT_LIMIT];
	bool photoTaken[OBJECT_LIMIT];
	for (int i = 0; i < OBJECT_LIMIT; i++) {
		radii[i] = sqrt(PIXELTHRESH);
		photoTaken[i] = false;
	}
	int loopCount = 0;
	int frame;

	while (!exitLawnmower && distance > WAYPOINT_RADIUS) {
		setLawnCourse(&course, distance, pastDistances, bearing, yaw);
		sprintf(str, "Course set to : {%d (A), %d (E)}", course.aileron, course.elevator);
		logPtr->writeLogLine(str);
		fbPtr->setFB_Data(&course);
		
		view = raspiCamCvQueryFrame(camPtr);
		BGRTemp = cvarrToMat(view);
		resize(BGRTemp, image, Size(COLSIZE, ROWSIZE), 0, 0, INTER_LINEAR);
		cvtColor(image, imHSV, CV_BGR2HSV);
		cvtColor(image, imBin, CV_BGR2GRAY);
		HSV2Bin(imHSV, imBin);
		loopCount++;
		if (loopCount%20 == 0) {
			nObjects = findRedObjects(imBin, centres);	//Finds centres
		}
		
		for (frame = 0; frame < nObjects; frame++) {
			if (radii[frame] != 0) {
				radii[frame] = sqrt(camShift(centres[frame], radii[frame], imBin));
				if ((centres[frame][1] < (2.0/3.0)*ROWSIZE) && (centres[frame][1] > (1.0/3.0)*ROWSIZE) && !photoTaken[frame]) {
					photoTaken[frame] = true;
					sprintf(str, "/home/pi/picopter/apps/photos/Lawnmower_lat_%d_ lon_%d_time_%d_obj_%d.jpg", (int)((dataPtr->latitude)*1000), (int)((dataPtr->longitude)*1000), (int)((dataPtr->time)*100), nObjects);
					imwrite(str, bestImg);
				}
			}
			else {
				centres[frame][0] = -1;
				centres[frame][1] = -1;
			}
		}
		
		delay(LOOP_WAIT);	//Wait for instructions
		gpsPtr->getGPS_Data(dataPtr);
		if (usingIMU) {
			imuPtr->getIMU_Data(compDataPtr);
			yaw = compDataPtr->yaw;	
			cout << "Yaw measured as: " << yaw << endl;
			sprintf(str, "Yaw measured	as %f", yaw);
			logPtr->writeLogLine(str);
		}
		updatePicture(oval, dataPtr->latitude, dataPtr->longitude);	
		namedWindow("Oval Map", CV_WINDOW_AUTOSIZE);
		imshow("Oval Map", oval);
		waitKey(1); 
		start.lat = (dataPtr->latitude);
		start.lon = (dataPtr->longitude);
		cout << "Needs to move from: " << dataPtr->latitude << " " << dataPtr->longitude << "\n\tto : " <<end.lat << " " << end.lon << endl;
		sprintf(str, "Currently at %f %f", dataPtr->latitude, dataPtr->longitude);
		logPtr->writeLogLine(str);
		sprintf(str, "%f %f %d", dataPtr->longitude, dataPtr->latitude, index);
		rawLogPtr->writeLogLine(str, false);
		sprintf(str, "Going to %f %f", end.lat, end.lon);
		logPtr->writeLogLine(str);
		logPtr->writeLogLine(str);
		logPtr->writeLogLine("\n");
		distance = calculate_distance(start, end);
		bearing = calculate_bearing(start, end);
		cout << "Distance: " << distance << " m\tBearing: " << bearing << endl;
		sprintf(str, "Distance: %f m\tBearing : %f degrees", distance, bearing);
		for (int i = 0; i < PAST_POINTS-1; i++) {
			pastDistances[i] = pastDistances[i+1];	//Shift down distance values
		}
		pastDistances[PAST_POINTS-1] = distance;	//Add on to the end
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
			if (((p[j] > HMIN) || (p[j] < HMAX)) && (p[j] > SMIN) && (p[j] < SMAX) && (p[j] > VMINIMUM) && (p[j] < VMAX)) {
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
			if (((p[j] > HMIN) || (p[j] < HMAX)) && (p[j] > SMIN) && (p[j] < SMAX) && (p[j] > VMINIMUM) && (p[j] < VMAX)) {
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
	if ((row - PIXEL_RADIUS) < 0) row = PIXEL_RADIUS;					//Check if we are going out of bounds of the image
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

void setLawnCourse(FB_Data *instruction, double distance, double pastDistances[], double bearing, double yaw) {
	double average = 0;
	for (int i = 0; i < PAST_POINTS; i++) {
		average = average + pastDistances[i];
	}
	average = average/PAST_POINTS;
	double speed = KPxy * distance + KIxy * average;
	if(speed > SPEED_LIMIT) {		//PI controler with limits.
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
	corners[3] = end;
	cout << "Corner #1 read as: " << (corners[0].lat) << " " << (corners[0].lon) << endl;
	sprintf(str, "Corner #1 read as: %f %f", (corners[0].lat), (corners[0].lon));
	logPtr->writeLogLine(str);	
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

	vector<Pos> sideA;
	vector<Pos> sideB;
	populateVector(corners[0], corners[1], &sideA);
	for(int i = 0; i < (int)sideA.size(); i++) {
		sprintf(str, "Point %d of sideA is %f %f", i+1, (sideA[i].lat), (sideA[i].lon));
		logPtr->writeLogLine(str);
	}
	populateVector(corners[2], corners[3], &sideB);
	for(int i = 0; i < (int)sideB.size(); i++) {
		sprintf(str, "Point %d of sideB is %f %f", i+1, (sideB[i].lat), (sideB[i].lon));
		logPtr->writeLogLine(str);
	}
	int minVectorLength = sideA.size();
	if ((int)sideB.size() < minVectorLength) minVectorLength = sideB.size(); //Checks which is smallest
	
	for (int i = 0; i < minVectorLength; i++) {
		if (i%2 == 0) {	//Even?
			//sprintf(str, "%d %d- Even", i,minVectorLength);
			//lawnlog.writeLogLine(str);
			list->push_back(sideA[i]);
			list->push_back(sideB[i]);
		}
		else if (i%2 == 1) {//Odd?
			//sprintf(str, "%d %d - Odd", i, minVectorLength);
			//lawnlog.writeLogLine(str);
			list->push_back(sideB[i]);
			list->push_back(sideA[i]);
		}
	}

	cout << "All waypoints calculated." << endl;
}

void populateVector(Pos start, Pos end, vector<Pos> *list) {
	int direction = 1;
	if (end.lon < start.lon) {
		direction = -1;	//Are we going E->W instead of W->E?
	}
	double endDistance = calculate_distance(start, end);	//Great circle distance, but ~ straight line distance for close points
	int points = (endDistance/SWEEP_SPACING);
	double fraction, distance, angle;
	list->push_back(start);
	for (int i = 1; i < points; i++) {
		fraction = (double)i/points;
		distance = fraction*endDistance;
		angle = distance/(RADIUS_OF_EARTH*cos((start.lat)*(PI/180)))*(180/PI);	//Both points have the same latitude
		Pos position;
		position.lat = start.lat;
		position.lon = start.lon + (double)direction*angle;
		list->push_back(position);
	}
	list->push_back(end);
}

void terminateLawn(int signum) {
	cout << "Signal " << signum << " received. Quitting lawnmower program." << endl;
	exitLawnmower = true;
}

//Merrick's stuff----------------------------------------------------------

void HSV2Bin(Mat &HSVImage, Mat (&binaryImage)){
	uchar* p;
	uchar* q;
	for(int i = 0; i < HSVImage.rows; i++) {
                p = HSVImage.ptr<uchar>(i);
		q = binaryImage.ptr<uchar>(i);
		for(int j = 0; j < HSVImage.cols; j++) {
			if(((p[3*j] > HMIN) || (p[3*j] < HMAX)) && (p[3*j+1] > SMIN) && (p[3*j+1]<SMAX) && (p[3*j+2] > VMINIMUM) && (p[3*j+2] < VMAX)){
				q[j] = WHITE;
			} else{
				q[j] = BLACK;
			}
		}
	}
	return;
}

int findRedObjects(Mat &binaryImage,  int (&redCentres)[OBJECT_LIMIT][2]) {
	//cout<<"1" <<endl;
	uchar* p;
	int count = 0;
	queue<vec2> que;
	int connComp[binaryImage.rows][binaryImage.cols];
	for(int i = 0; i<binaryImage.rows; i++){
		for(int j = 0; j<binaryImage.cols; j++){
			connComp[i][j]=0;
		}
	}
	
	vec2 temp;
	vec2 temp2;
	int label = 1;
	int pixCount[OBJECT_LIMIT];
	Mat elementErode(ERODE_ELEMENT,ERODE_ELEMENT,CV_8U,Scalar(255));
	Mat elementDilate(DILATE_ELEMENT,DILATE_ELEMENT,CV_8U,Scalar(255));
	dilate(binaryImage,binaryImage,elementDilate);
	erode(binaryImage,binaryImage,elementErode);
	for(int i = 0; i < binaryImage.rows; i++) {
		p = binaryImage.ptr<uchar>(i);
		for(int j = 0; j < binaryImage.cols; j++) {
			if(p[j] == WHITE) {
				if(connComp[i][j]==0){
					connComp[i][j]=label;///
					temp.a=i;
					temp.b=j;
					que.push(temp);
					pixCount[label-1] = 1;
					while(!que.empty()){
						temp.a=que.front().a;
						temp.b=que.front().b;
						que.pop();
						if((temp.a<binaryImage.rows-1)&&(connComp[temp.a+1][temp.b]==0)&&(binaryImage.ptr<uchar>(temp.a+1)[temp.b]==WHITE)){
							connComp[temp.a+1][temp.b]=label;
							temp2.a=temp.a+1;
							temp2.b=temp.b;
							que.push(temp2);
							pixCount[label-1]++;
						}
						if((temp.b<binaryImage.cols-1)&&(connComp[temp.a][temp.b+1]==0)&&(binaryImage.ptr<uchar>(temp.a)[temp.b+1]==WHITE)){
                                                        connComp[temp.a][temp.b+1]=label;
                                                        temp2.a=temp.a;
							temp2.b=temp.b+1;
                                                        que.push(temp2);
							pixCount[label-1]++;
                                                }
						if((temp.b>0)&&(connComp[temp.a][temp.b-1]==0)&&(binaryImage.ptr<uchar>(temp.a)[temp.b-1]==WHITE)){
                                                        connComp[temp.a][temp.b-1]=label;
                                                        temp2.a=temp.a;
							temp2.b=temp.b-1;
                                                        que.push(temp2);
                                                	pixCount[label-1]++;
						}
						if((temp.a>0)&&(connComp[temp.a-1][temp.b]==0)&&(binaryImage.ptr<uchar>(temp.a-1)[temp.b]==WHITE)){
                                                        connComp[temp.a-1][temp.b]=label;
                                                        temp2.a=temp.a-1;
							temp2.b=temp.b;
                                                        que.push(temp2);
							pixCount[label-1]++;
                                                }
					}
					label++;
					if (label>OBJECT_LIMIT) break;
				}
			}
		}
	}
	
	int pixelCount;
	int xCentre;
	int yCentre;
	for(int k = 1; k<label; k++){
		if(pixCount[k-1]<PIXELTHRESH) continue;
		pixelCount= 0;
		xCentre = 0;
		yCentre = 0;
		for(int i = 0; i < binaryImage.rows; i++){
			for(int j = 0; j <binaryImage.cols; j++){
				if(connComp[i][j]==k){
					pixelCount++;
					xCentre += j;
					yCentre += i;
				}
			}
		}
		if (pixelCount > PIXELTHRESH) {
			redCentres[count][0] = xCentre/pixelCount;
			//cout<<redCentres[k-1][0]<<endl;
			//cout<<binaryImage.rows<<endl;
			redCentres[count][1] = yCentre/pixelCount;	
			//cout<<redCentres[k-1][1]<<endl;
			//cout<<binaryImage.cols<<endl;
			count++;
		}
	}
	return count;
}
int camShift(int (&centre)[2], int size, Mat binImage) {
	int x;
	int y;
	int xCentre = 0;
	int yCentre = 0;
	int pixelCount=0;
	uchar* p;
	for(int i=0-size; i<size; i++) {
		y = centre[1] + i;
		if((y>=0)&&(y<binImage.rows)){
			p=binImage.ptr<uchar>(y);
			for(int j = 0-size; j<size;j++) {
				x = centre[0] + j;
				if((i*i+j*j <= size*size) && (x>=0)&&(x<binImage.cols)&&(p[x]==WHITE)) {
					xCentre += x;
					yCentre += y;
					pixelCount++;
				}
			}
		}
	}
	//cout<<xCentre<<" "<<yCentre<<" "<<pixelCount<<endl;
	if (pixelCount>0) {
	centre[0]=xCentre/pixelCount;
	centre[1]=yCentre/pixelCount;
	cout<<centre[0]<<", "<<centre[1]<<endl;
	return pixelCount;
	} else return 0;	
}


void runDetection(RaspiCamCvCapture *capture) {
	//capture image
	//RaspiCamCvCapture *capture = raspiCamCvCreateCameraCapture(0); // Index doesn't really matter
	cvNamedWindow("RaspiCamTest", 0);
	int centres[OBJECT_LIMIT][2];
	Mat binImg;
	Mat imHSV;
	IplImage* image;
	Mat BGRImage;
	Mat BGRTemp;
	int numObjects;
	int radii[OBJECT_LIMIT];
	int frame;
	timespec ts;
	timespec bs;
	for (int k = 0; k < OBJECT_LIMIT; k++) {
		radii[k]=20;
                centres[k][0]=-1;
                centres[k][1]=-1;
        }
	
	while(cvWaitKey(10) < 0){
		clock_gettime(CLOCK_REALTIME, &ts);
		
		image = raspiCamCvQueryFrame(capture);
		BGRTemp =cvarrToMat(image);
		resize(BGRTemp, BGRImage, Size(160,120),0, 0, INTER_LINEAR);
		//imshow("RaspiCamTest",BGRImage);
		
		cvtColor(BGRImage,imHSV,CV_BGR2HSV);
		
		cvtColor(BGRImage,binImg,CV_BGR2GRAY);
		HSV2Bin(imHSV,binImg);
		numObjects = findRedObjects(binImg,centres);
		cout<<"Red Objects Detected:"<<numObjects<<endl;
		for (frame=0; frame < numObjects; frame++) {
			if (radii[frame] != 0) {
				radii[frame] = sqrt(camShift(centres[frame] , radii[frame]  ,binImg));
			}
			else {
				centres[frame][0] = -1;
				centres[frame][1] = -1;
			}
		}
		imshow("RaspiCamTest",BGRImage);
		namedWindow("Connected Components");
		imshow("Connected Components",binImg);
		clock_gettime(CLOCK_REALTIME,&bs);
		cout<<1000.0/((bs.tv_nsec-ts.tv_nsec)/1000000)<<" fps"<<endl;
	}

	cvDestroyWindow("RaspiCamTest");
	//raspiCamCvReleaseCapture(&capture);
}
