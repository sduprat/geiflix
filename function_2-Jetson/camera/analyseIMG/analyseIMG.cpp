#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

#include <list>

#include <chrono>
#include <opencv2/highgui/highgui_c.h>



using namespace cv;
using namespace std;


void DrawCircle(int posx, int posy, Mat & m, bool isMain) {
	if (isMain){ circle(m, Point(posx, posy), 60, 150, 20); }
	else{ circle(m, Point(posx, posy), 20, 150, 5); }
	circle(m, Point(posx, posy), 12, 70, 2);
	

}

void whiteGroups(Mat& m, int& lastMain, int& tries, bool useLastOne) {
	int Tolerance = 1, minDetec = m.cols, minWidth = m.cols / 20, maxWidth = m.cols / 1.5;
	int lastWhite = 0, nbInGroup = 0, sum = 0, first = 0;
	//-------------------------------Find the right white circle
	int maxSize = 0, mainIndex = 0, maxChange = m.cols / 10;
	//-------------------------------
	list<int> WhitePxList, Circles, sizeCircles;
	// This part create a list of x's value from all white px on the threshold image
	uchar* ptr;
	int i, j;
	for (i = m.rows-1; i > -1; i--) {

		ptr = m.ptr<uchar>(i);

		for (j = m.cols-1; j > -1; j--)
		{
			if (ptr[j] == 255) {
				WhitePxList.push_back(j);
			}
		}
	}
	WhitePxList.sort(); // we sort them


	// We use this list to make a new list of groups of white px detected on the picture
	first = WhitePxList.back();
	list<int>::iterator it = WhitePxList.end();
	for (i = WhitePxList.size();i > -1;i--) {

		if ((lastWhite - *it) > Tolerance)
		{
			if (nbInGroup > minDetec && (first-*it)>minWidth && (first-*it)<maxWidth) { Circles.push_back(sum / nbInGroup); sizeCircles.push_back(nbInGroup);}
			nbInGroup = 0;
			sum = *it;
			first = sum;
		}
		else
		{
			sum += *it;
			nbInGroup += 1;
		}
		lastWhite = *it;
		it--;
	}
	if (nbInGroup > minDetec) { Circles.push_back(sum / nbInGroup); sizeCircles.push_back(nbInGroup); }
	//Find Main white groups
	//Here we have two main methods, the first will check for the white group which is the closest from the last value measured
	//The second one will focus on the biggest white group avaiable
	//Most of the time we will use the first method, however if we cannot find a white group close enough from the last value, we will use the last value measured
	//If we can't find a group close enough for a chosen number of itteration, we will use the second method to chose a new white group to follow
	if (useLastOne) {
		it = Circles.begin();
		int difMin = abs(lastMain-*it);
		mainIndex = 0;

		for (int i = 1; i < Circles.size(); i++) {
			it++;
			if (abs(lastMain - *it) < difMin) { difMin = abs(lastMain - *it);mainIndex = i; }
		}

		if (difMin > maxChange) //test if there is a white group close enough from last value measured
		{
		if (tries > 100) {useLastOne = false;} //keep track of the number of time we failed
			else { // use last value measured if we can't find a white group close enough
				Circles.push_back(lastMain);
				mainIndex = Circles.size() -1;
			}
			tries++;
		}else { tries = 0; }
		
	}
	if (!useLastOne){
	//else {
		it = sizeCircles.begin();
		for (int i = 0; i < sizeCircles.size();i++) {
			if (*it > maxSize) { maxSize = *it; mainIndex = i; }
			it++;
		}
	}
	//We draw these white groups 
	it = Circles.begin();
	for (int i = 0; i < Circles.size(); i++) { 
		if(i==mainIndex){ lastMain = *it; }
		DrawCircle(*it, m.rows / 2, m, i==mainIndex); 
		it++; 
	}
}
void testImageAnalysis(bool useCam, Mat & Img1, int& lastPos, int& tries){
	//Chrono initialization to evaluate performances
	using std::chrono::high_resolution_clock;
	using std::chrono::duration_cast;
	using std::chrono::duration;
	using std::chrono::milliseconds;

	auto t1 = high_resolution_clock::now();


	// Here, we read an image in grayscale, use a threshold function on it to only have white and black pixels, send this threshold picture to our function
	if (!useCam) {
		Img1 = imread("C:\\Users/celes/Desktop/cours INSA 5A s1/projet voiture/opencvtest/TestImages/lvl4.png", IMREAD_GRAYSCALE);
		imshow("testAFF", Img1);
	}
	threshold(Img1, Img1, 210, 255, 0);
	//imshow("testAFF2", Img1);

	//whiteGroups(Img1, 0, false);
	whiteGroups(Img1, lastPos, tries, true);
	imshow("testAFF3", Img1);

	auto t2 = high_resolution_clock::now();

	/* Getting number of milliseconds as an integer. */
	auto ms_int = duration_cast<milliseconds>(t2 - t1);
	//std::cout << ms_int.count() << "ms\n";

	//waitKey(0);
	//Img1.release();
}

int main() {
	int lastPos = 300;
	int tries = 0;
	// this will contain the image from the webcam
	Mat frame;
	//testImageAnalysis();
	cv::VideoCapture camera(0);
	if (!camera.isOpened()) {
		std::cerr << "ERROR: Could not open camera" << std::endl;
		return 1;
	}
	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

	while (1) {
		// show the image on the window
		// capture the next frame from the webcam
		camera >> frame;
		cvtColor(frame, frame, COLOR_BGR2GRAY); //Convert into grayscale
		imshow("Webcam", frame);

		// wait (1ms) for a key to be pressed to exit loop
		if (waitKey(1) >= 0)
			break;

		testImageAnalysis(true, frame, lastPos, tries);
	}


	//release memory and end program
	frame.release();
	return 0;
}



