#include "detectObjects.h"

typedef uchar uchar;
typedef struct vec2{int a; int b;} vec2;

int HMIN = 320;
int HMAX = 40;
int SMIN=  95;
int SMAX = 255;
int VMINIMUM = 95;
int VMAX = 255;
int WHITE = 255;
int BLACK = 0;
int COLSIZE = 160;
int ROWSIZE = 120; 
int PIXELTHRESH = 12;
int DILATE_ELEMENT = 6;
int ERODE_ELEMENT = 6;

void loadCameraConfig(void) {
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
}

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

void runTrackObject(FlightBoard* fb){
	RaspiCamCvCapture *capture = raspiCamCvCreateCameraCapture(0);
	int centre[2];
	centre[0]=160;
	centre[1]=120;
	int radii=160;
	Mat binImg;
	Mat imHSV;
	timespec ts;
	IplImage* image;
	Mat BGRImage;
	Mat BGRTemp;
	FB_Data stop = {0, 0, 0, 0};
	FB_Data rotate = {0, 0, 20, 0};
	FB_Data rotateLeft = {0, 0, -10, 0};
	FB_Data rotateRight = {0, 0, 10, 0};
	FB_Data forward = {0, 20, 0, 0};
	FB_Data backward = {0, -20, 0, 0};
	while(cvWaitKey(10) < 0) {
		clock_gettime(CLOCK_REALTIME, &ts);
		
		image = raspiCamCvQueryFrame(capture);
		BGRTemp =cvarrToMat(image);
		resize(BGRTemp, BGRImage, Size(320,240),0, 0, INTER_LINEAR);
		cvtColor(BGRImage,imHSV,CV_BGR2HSV);
		cvtColor(BGRImage,binImg,CV_BGR2GRAY);
		HSV2Bin(imHSV,binImg);
		radii = sqrt(camShift(centre,radii,binImg));
		if (radii==0) {
			fb->setFB_Data(&rotate);
			radii=360;
		}
		else {
			if(centre[0] < 106){
				 fb->setFB_Data(&rotateRight);
			}
			else if(centre[0] > 213) {
				fb->setFB_Data(&rotateLeft);
			} else fb->setFB_Data(&stop);
			if(centre[1] < 80) {
				fb->setFB_Data(&forward);
			}
			else if(centre[1] >160) {
				fb->setFB_Data(&backward);
			} //else fb->setFB_Data(&stop);
		}
	}
	raspiCamCvReleaseCapture(&capture);
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

//Old image processing functions
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
