//
//  main.cpp
//  video_target
//
//

#include "stdafx.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <thread>
#include <math.h>
#include <time.h>
#include <mutex>
#include <stdio.h>
#include <windows.h>
#include "boost/thread/thread.hpp"
#include "boost/algorithm/clamp.hpp"
#include "AimAssist.h"




/*  Define the API model (PLUGIN or DIRECT) before including the
*  gcapi.h header file.
*/
#define GCAPI_DIRECT
#include "gcapi.h"

/*  Direct API exported functions. Check gcapi.h for a more detailed
*  description.
*/
GCDAPI_Load gcdapi_Load = NULL;
GCDAPI_Unload gcdapi_Unload = NULL;
GCAPI_IsConnected gcapi_IsConnected = NULL;
GCAPI_GetFWVer gcapi_GetFWVer = NULL;
GCAPI_Read gcapi_Read = NULL;
GCAPI_Write gcapi_Write = NULL;
GCAPI_GetTimeVal gcapi_GetTimeVal = NULL;
GCAPI_CalcPressTime gcapi_CalcPressTime = NULL;


std::mutex mtx, mtx2, mtx3, mtx4;
int xres = 1920;
int yres = 1080;
int fps = 30;
double delay = 1000.00 / (double)fps;
cv::Mat ViewImage = cv::Mat::zeros(1080, 1920, CV_8UC3);
int grx(0), gry(0), gr2(0), gsec(1);
bool Targeting;

// SENSI 5.
int iSliderValue1 = 79; //Speed
int iSliderValue2 = 30; //Shoot Distance
int iSliderValue3 = 68; //HeadLoc decal
int iSliderValue4 = 150; //YX ratio
int iSliderValue6 = 65; //Min Speed
bool TriggerMode = FALSE;
int rrx, rry;
double residue_x = 0.0;
double residue_y = 0.0;
double gdist;
auto TaTimerStart = std::chrono::high_resolution_clock::now();
auto TaTimerEnd = std::chrono::high_resolution_clock::now();
auto time_counter = std::chrono::high_resolution_clock::now() ;
using namespace cv;
using namespace std;

// XY ratio
// Pixel Distance to stop moving
// Color flag for different mode
// Decal flag to find sweet spot.

// Implement Primary detector using central red dot on most weapons.
// IsPrimary() & IsSecondary()

// Implement ADS detector using L2 input on TitanOne.exe side.
// merge both Exe and boost::threads..for efficiency.



// Experiment2: try a 2 stages move => on Pixel then go down.

//hide the local functions in an anon namespace
namespace {



	bool isPrimary(Mat roi) {
		return 0;
	}

	bool isSecondary(Mat roi) {
		return 0;
	}

	bool isAimingDownSight(Mat roi) {
		return 0;
	}


	float guessZoomLevel(int wpixel) {
		float res = float(wpixel) / float(400.00);
		return res;
	}


	vector<int>  polarDZ(double io_x, double io_y, double dz, double distance) {


		double low = (double)iSliderValue1;
		double high = 80.00;



		double rx = io_x ;
		double ry = io_y * double(iSliderValue4)/100.00;

		
		double dd = double(iSliderValue1)/10.0*log(distance*distance);
		dd = boost::algorithm::clamp(dd, double(iSliderValue6), 97.00);
		if (dd != dd) dd = 0.00;
		double angle = atan2(ry, rx);



		double radius;

		if ((rx == 0 && ry == 0)) {
			radius = 0;
	
		}
		else {

			radius =  dd;
		}


		double OutX = cos(angle) * radius  ;
		double OutY = sin(angle) * radius  ;
		if (OutX != OutX) OutX = 0.00;
		if (OutY != OutY) OutY = 0.00;
		
		OutX = boost::algorithm::clamp(OutX , -100.00, 100.00);
		OutY = boost::algorithm::clamp(OutY , -100.00, 100.00);

		return { int(OutX),int(OutY) };
	}







	vector<Point>  findLeftMostPixel(Mat m) {
		Mat nonZeroCoordinates;
		findNonZero(m, nonZeroCoordinates);
		int min_x(m.cols);
		int max_x(0);
		int min_y(m.rows);
		int max_y(0);
		bool found(false);

		for (int i = 0; i < int(nonZeroCoordinates.total()); i++) {
			if (nonZeroCoordinates.at<Point>(i).x < min_x) {
				min_x = nonZeroCoordinates.at<Point>(i).x;
				found = true;
			}
			else {
				max_x = nonZeroCoordinates.at<Point>(i).x;
			}
			if (nonZeroCoordinates.at<Point>(i).y < min_y) {
				min_y = nonZeroCoordinates.at<Point>(i).y;
				found = true;
			}
			else {
				max_y = nonZeroCoordinates.at<Point>(i).y;
			}

		}
		vector<Point> result;
		if (!found) return{ Point(0,0),Point(0,0) };
		result.push_back(Point(min_x, min_y));
		result.push_back(Point(max_x, max_y));

		return result;
	}

	



	int TitanOneDI() {

		HINSTANCE hInsGPP = NULL;
		GCAPI_REPORT report = { 0 };
		int8_t output[GCAPI_OUTPUT_TOTAL] = { 0 };

		// Load the Direct API Library
		hInsGPP = LoadLibrary(TEXT("gcdapi.dll"));
		if (hInsGPP == NULL) {
			MessageBox(NULL, TEXT("Error on loading gcdapi.dll"), TEXT("Error"), MB_OK);
			return(1);
		}

		// Set up the pointers to DLL exported functions
		gcdapi_Load = (GCDAPI_Load)GetProcAddress(hInsGPP, "gcdapi_Load");
		gcdapi_Unload = (GCDAPI_Unload)GetProcAddress(hInsGPP, "gcdapi_Unload");
		gcapi_IsConnected = (GCAPI_IsConnected)GetProcAddress(hInsGPP, "gcapi_IsConnected");
		gcapi_GetFWVer = (GCAPI_GetFWVer)GetProcAddress(hInsGPP, "gcapi_GetFWVer");
		gcapi_Read = (GCAPI_Read)GetProcAddress(hInsGPP, "gcapi_Read");
		gcapi_Write = (GCAPI_Write)GetProcAddress(hInsGPP, "gcapi_Write");
		gcapi_GetTimeVal = (GCAPI_GetTimeVal)GetProcAddress(hInsGPP, "gcapi_GetTimeVal");
		gcapi_CalcPressTime = (GCAPI_CalcPressTime)GetProcAddress(hInsGPP, "gcapi_CalcPressTime");

		if (gcdapi_Load == NULL || gcdapi_Unload == NULL || gcapi_IsConnected == NULL || gcapi_GetFWVer == NULL ||
			gcapi_Read == NULL || gcapi_Write == NULL || gcapi_GetTimeVal == NULL || gcapi_CalcPressTime == NULL) {
			FreeLibrary(hInsGPP);
			MessageBox(NULL, TEXT("Error on gcdapi.dll"), TEXT("Error"), MB_OK);
			return(1);
		}

		// Allocate resources and initialize the Direct API.
		// You must call this function before use the API.
		uint8_t loaded = gcdapi_Load();
		Sleep(100);
		if (!loaded) {
			FreeLibrary(hInsGPP);
			MessageBox(NULL, TEXT("Unable to initiate the Direct API"), TEXT("Error"), MB_OK);
			return(1);
		}

		double timer = 0.00;
		while (1) {
			auto start = chrono::high_resolution_clock::now();

			int rx = 0;
			int ry = 0;
			int r2 = 0;


			if (timer >= 10) {
				   timer = 0.00;
					mtx2.lock();
					rx = grx;  ry = gry;  r2 = gr2;
					mtx2.unlock();

					if (gcapi_IsConnected()) {
				

						gcapi_Read(&report);

						for (uint8_t i = 0; i < GCAPI_INPUT_TOTAL; i++) {
							output[i] = report.input[i].value;
						}

					
						if (rx == 0 || abs(report.input[PS4_RX].value) > 10) {
							output[PS4_RX] = report.input[PS4_RX].value;
						}
						else {
							output[PS4_RX] = rx;

						}

						if (ry == 0 || abs(report.input[PS4_RY].value) > 10) {
							output[PS4_RY] = report.input[PS4_RY].value;
						}
						else {
							output[PS4_RY] = ry;
						}
						if (r2 == 0 || abs(report.input[PS4_R2].value) > 10) {
							output[PS4_R2] = report.input[PS4_R2].value;
						}
						else output[PS4_R2] = r2;
						gcapi_Write(output);
			
					} 



			}
			auto end = chrono::high_resolution_clock::now();
			auto diff = end - start;
			double loopduration = chrono::duration <double, milli>(diff).count();
			timer += loopduration;
			//cout << "TitanOne Thread:                  " << loopduration << " ms" << endl;
		}




		// Free API resources and unload the library.
		gcdapi_Unload();
		FreeLibrary(hInsGPP);

		return 0;

	}

	int process1(VideoCapture& capture, Mat& ViewImage) {

		int n = 0;

		//  Detect RED color RBG (189, 54, 43)  HP bar color..
		// Training object RGB ()

		int r = 182; //189 //182
		int g = 44;  //54  //44
		int b = 34;  //43  //34


		// maximum color distance
		int v = 1;
		// Reticle coordinate in RoI
		double ret_y = double(yres) / 1.736;
		Point aimLoc = Point(int(xres / 2), int(ret_y));

		// define the Region Of Interest (roi)

		int roi_x = xres / 4; // (547,247) optimum for to avoid HUD
		int roi_dx = 2 * (aimLoc.x - roi_x);

		int roi_y = yres/4;
		int roi_dy = 2 * (aimLoc.y - roi_y);


		
		
		//float max_d = sqrt(pow(float(aimLoc.x - roi_x), 2) + pow(float(aimLoc.y - roi_y), 2));

		// Head decals TODO: need to make it adaptive to ennemies distance..
		// need more footages at various distances..

		int h_decal_x = -8;
		int h_decal_y = iSliderValue3; 



		Rect roi(roi_x, roi_y, roi_dx, roi_dy);

		for (;;) {
			h_decal_y = iSliderValue3;

			auto start = chrono::high_resolution_clock::now();

			Mat colored_img;
			Mat frame;
			Mat gray;
			clock_t startTime = clock();


			if (!capture.read(frame))
				break;
			colored_img = frame.clone();

			// Processing :


			// Find Color:
			// TODO:
			//
			// - get L2 state to activate aim only when ADS..
			// - have a toggle for Long & Mid Range / Close Range for h_decal


			Mat work;
			work = frame(roi).clone();

			cv::Scalar lowerb = cv::Scalar(b - v, g - v, r - v);
			cv::Scalar upperb = cv::Scalar(b + v, g + v, r + v);

			cv::inRange(work, lowerb, upperb, gray);
			cv::medianBlur(gray, gray, 5);

			// option 1: evaluate number of white pixel to guess distance and adjust h_decal
			// option 2: evaluate find HP bar pixel and Level icon red pixel and from distance derive zoom/h_decal

			//int white = cv::countNonZero(gray);



			// Find coordinate of 1st white pixel in gray image with minMaxLoc.
			Point minLoc, maxLoc, matchLoc;
			vector<Point> Boundary;



			Boundary = findLeftMostPixel(gray);
			matchLoc = Boundary[0];
			int HPheight = Boundary[1].y - Boundary[0].y;
			int HPWidth = Boundary[1].x - Boundary[0].x;
			int HParea = HPheight * HPWidth;



			Point headLoc;

			int shoot = 0;
			double distan = 1000;





			if (matchLoc.x != 0 && matchLoc.y != 0) {


				headLoc = cv::Point(matchLoc.x + roi_x + h_decal_x, matchLoc.y + roi_y + h_decal_y);

				distan = cv::norm(aimLoc - headLoc);

				if (distan <= iSliderValue2) {
					shoot = 100;
				}

				// draw circle on head
				cv::circle(colored_img, headLoc, 2, CV_RGB(0, 0, 255), 2);

				// draw line to reticle
				cv::line(colored_img, headLoc, aimLoc, CV_RGB(255, 0, 0), 3);
			}


			// Draw Active Region of Interest

			cv::rectangle(
				colored_img,
				cv::Point(roi_x, roi_y),
				cv::Point(roi_x + roi_dx, roi_y + roi_dy),
				CV_RGB(0, 255, 0),
				3);




			int RX(0), RY(0), R2(0);
			if (headLoc != Point(0, 0)) {

				RX = (headLoc.x - aimLoc.x);
				RY = (headLoc.y - aimLoc.y);
				R2 = shoot;
			}

			//vector<int> move = { 0,0 };
			//if (!TriggerMode)
			vector<int> move = polarDZ(RX, RY, 24.00,distan);
			mtx2.lock();
			grx = move[0];
			gry = move[1];
			gr2 = R2;
			gdist = distan;
			mtx2.unlock();





			mtx.lock();
			ViewImage = colored_img.clone();
			mtx.unlock();
			auto end = chrono::high_resolution_clock::now();
			auto diff = end - start;
			double loopduration = chrono::duration <double, milli>(diff).count();
			//cout << "Capture Thread:                  " << loopduration << " ms" << endl;

		}
		return 0;
	}
}

int main() {



	VideoCapture capture;

	capture.open(700); //DSHOW
	capture.set(3, xres);
	capture.set(4, yres);
	capture.set(5, fps);
	capture.set(6, CV_FOURCC('Y', 'U', 'Y', '2'));

	//if this fails, exit 
	if (!capture.isOpened()) {
		cerr << "Failed to open the video device!\n" << endl;

		return 1;
	}
	namedWindow("Control", WINDOW_AUTOSIZE); // control window;
	namedWindow("Image", WINDOW_AUTOSIZE); // control window;

	createTrackbar("Move Speed", "Control", &iSliderValue1, 200);
	createTrackbar("Shoot Distance", "Control", &iSliderValue2, 100);
	createTrackbar("Head Delta", "Control", &iSliderValue3, 1000);
	createTrackbar("YX Ratio", "Control", &iSliderValue4, 200);
	createTrackbar("Min Speed", "Control", &iSliderValue6, 100);

	// Processing thread has started
	boost::thread ProcessThread(process1, boost::ref(capture), boost::ref(ViewImage));
	// TitanOne thread has started
	boost::thread TitanOneThread(TitanOneDI);

	Mat Resized;

	while (1) {
		mtx.lock();
		Resized = ViewImage.clone();
		mtx.unlock();
		if (1) {
			cv::resize(Resized, Resized, cv::Size(), 0.25, 0.25);
			imshow("Image", Resized);
		}
		char key = (char)waitKey(8);

		switch (key) {
		case 'q':
		case 'Q':
		case 27: //escape key
			return 0;
		default:
			break;
		}

	}
	ProcessThread.join();
	TitanOneThread.join();
	capture.release();
	return 1;

}

