#include "detectObjects.h"

void HSV2Bin(Mat HSVImage, Mat (&binaryImage)){
	uchar* p;
	uchar* q;
	for(int i = 0; i < HSVImage.rows; i++) {
                p = HSVImage.ptr<uchar>(i);
		q = binaryImage.ptr<uchar>(i);
		//cout<<p[1]<<endl;
		//cout<<"1" <<endl;
                for(int j = 0; j < HSVImage.cols; j++) {
			//cout<<"2" <<endl;
			if(((p[3*j] > HMIN) || (p[3*j] < HMAX)) && (p[3*j+1] > SMIN) && (p[3*j+1]<SMAX) && (p[3*j+2] > VMIN) && (p[3*j+2] < VMAX)){
				//cout<<"3" <<endl;
				q[j] = WHITE;
			} else{
				//cout<<"4"<<endl;
				//cout<<i<<endl;
				//cout<<j<<endl;
				q[j] = BLACK;
			}
		}
	}
	return;
}

int findRedObjects(Mat &binaryImage,  int (&redCentres)[2][10]) {
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
	//cout<<"2"<<endl;
	//cout<<"initialise conncomp works" <<endl;
	vec2 temp;
	vec2 temp2;
	int label = 1;
	Mat element(ELEMENT_SIZE, ELEMENT_SIZE, CV_8U, Scalar(255));
	dilate(binaryImage,binaryImage,element);
	erode(binaryImage,binaryImage,element);
	//cout<<"pp"<<endl;
	for(int i = 0; i < binaryImage.rows; i++) {
		p = binaryImage.ptr<uchar>(i);
		for(int j = 0; j < binaryImage.cols; j++) {
			if(p[j] == WHITE) {
				if(connComp[i][j]==0){
					//cout<<"pixel"<<endl;
					connComp[i][j]=label;///
					temp.a=i;
					temp.b=j;
					que.push(temp);
					while(!que.empty()){
						temp.a=que.front().a;
						temp.b=que.front().b;
						que.pop();
						if((temp.a<binaryImage.rows-1)&&(connComp[temp.a+1][temp.b]==0)&&(binaryImage.ptr<uchar>(temp.a+1)[temp.b]==WHITE)){
							connComp[temp.a+1][temp.b]=label;
							temp2.a=temp.a+1;
							temp2.b=temp.b;
							que.push(temp2);
						}
						if((temp.b<binaryImage.cols-1)&&(connComp[temp.a][temp.b+1]==0)&&(binaryImage.ptr<uchar>(temp.a)[temp.b+1]==WHITE)){
                                                        connComp[temp.a][temp.b+1]=label;
                                                        temp2.a=temp.a;
							temp2.b=temp.b+1;
                                                        que.push(temp2);
                                                }
						if((temp.b>0)&&(connComp[temp.a][temp.b-1]==0)&&(binaryImage.ptr<uchar>(temp.a)[temp.b-1]==WHITE)){
                                                        connComp[temp.a][temp.b-1]=label;
                                                        temp2.a=temp.a;
							temp2.b=temp.b-1;
                                                        que.push(temp2);
                                                }
						if((temp.a>0)&&(connComp[temp.a-1][temp.b]==0)&&(binaryImage.ptr<uchar>(temp.a-1)[temp.b]==WHITE)){
                                                        connComp[temp.a-1][temp.b]=label;
                                                        temp2.a=temp.a-1;
							temp2.b=temp.b;
                                                        que.push(temp2);
                                                }
					}
					//cout<<j<<endl;
					label++;
				}
			}
		}
	}
	//cout<<"runnning conncomp worked" <<endl;
	
	int pixelCount;
	int xCentre;
	int yCentre;
	for(int k = 1; k<label; k++){
		if(k>10) break;
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
			redCentres[0][count] = xCentre/pixelCount;
			//cout<<redCentres[0][k-1]<<endl;
			//cout<<binaryImage.rows<<endl;
			redCentres[1][count] = yCentre/pixelCount;	
			//cout<<redCentres[1][k-1]<<endl;
			//cout<<binaryImage.cols<<endl;
			count++;
		}
	}
	return count;
}
int meanShift(int (&centre)[2], int size, Mat binImage){
	int x;
	int y;
	int xCentre;
	int yCentre;
	int pixelCount;
	uchar* p;
	for(int i=0-size; i<size; i++) {
		y = centre[1] + i;
		if((y>=0)&&(y<binImage.rows)){
			p=binImage.ptr<uchar>(y);
			for(int j = 0-size; j<size;j++) {
				x = centre[0] + j;
				if((i*i+j*j <= size*size) && (x>=0)&&(x<binImage.cols)&&(p[x]==WHITE)) {
					xCentre;
					yCentre;
					pixelCount++;
				}
			}
		}
	}
	centre[0]=xCentre/pixelCount;
	centre[1]=yCentre/pixelCount;
	return pixelCount;	
}

/*
int main() {
	//capture image
	cout<<"starting"<<endl;
	RaspiCamCvCapture *capture = raspiCamCvCreateCameraCapture(0); // Index doesn't really matter
	cout<<"capture works"<<endl;
	cvNamedWindow("RaspiCamTest", 0);
	cout<<"Window Works"<<endl;
	while(cvWaitKey(10) < 0){
	IplImage* image = raspiCamCvQueryFrame(capture);
	cv::Mat BGRImage = cv::cvarrToMat(image);
	imshow("RaspiCamTest",BGRImage);
	//cout<<"conv to Mat works"<<endl;
	Mat imHSV;
	cvtColor(BGRImage,imHSV,CV_BGR2HSV);
	Mat binImg;
	cvtColor(BGRImage,binImg,CV_BGR2GRAY);
	HSV2Bin(imHSV,binImg);
	cout<<binImg.cols<<endl;
	cout<<imHSV.cols<<endl;
	cout<<imHSV.rows<<endl;
	cout<<"conv to Binary works"<<endl;
	//Test Matrix: Mat binImg = Mat(90,90, CV_8UC1, cvScalar(255));
	int centres[2][10];
	for( int k=0; k<10; k++){
		centres[0][k]=-1;
		centres[1][k]=-1;
	}
	//cout<<"populate centres works"<<endl;
	int numObjects = findRedObjects(binImg,centres);
	cout<<"find red works"<<endl;
	cout<<numObjects<<endl;
	for (int frames=0;frames < numObjects; frames++) {
		
	}
	//while (cvWaitKey(10) < 0){
		imshow("RaspiCamTest",BGRImage);
		namedWindow("Connected Components");
		imshow("Connected Components",binImg);
	}

	cvDestroyWindow("RaspiCamTest");
	raspiCamCvReleaseCapture(&capture);
	
	return 0;
}
*/