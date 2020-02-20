// TitanOne.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include "zmq.hpp"
#include <windows.h>
#include <string>
#include <sstream>
#include <math.h>
#include <time.h>


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



int main()
{
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




	zmq::context_t ctx(1);

	zmq::socket_t sock(ctx, ZMQ_PULL);

	sock.bind("tcp://127.0.0.1:12345");

	zmq::message_t msg;
	while (1) {
		if (sock.recv(&msg, ZMQ_DONTWAIT)) {
			int rx = 0;
			int ry = 0;
			int r2 = 0;

			std::istringstream iss(static_cast<char*>(msg.data()));
			iss >> rx >> ry >> r2;

			std::cout << "RX:" << rx << std::endl;
			std::cout << "RY:" << ry << std::endl;
			std::cout << "R2:" << r2 << std::endl;



			// Basic usage example. 
			// Check if the GPP/Cronus is connected.
			if (gcapi_IsConnected()) {
				// The plugin is responsible for setting the output data. The loop that 
				// follows is only copying the input current states (from controller) to 
				// the output data struct (to console).
				clock_t startTime = clock();
				gcapi_Read(&report);

				for (uint8_t i = 0; i < GCAPI_INPUT_TOTAL; i++) {
					output[i] = report.input[i].value;
				}

				if (abs(report.input[PS4_L2].value) > 50) { //ADS

					if (rx == 0 || abs(report.input[PS4_RX].value) > 10) {
						output[PS4_RX] = report.input[PS4_RX].value;
					}
					else output[PS4_RX] = rx;

					if (ry == 0 || abs(report.input[PS4_RY].value) > 10) {
						output[PS4_RY] = report.input[PS4_RY].value;
					}
					else output[PS4_RY] = ry;
					if (r2 == 0 || abs(report.input[PS4_R2].value) > 10) {
						output[PS4_R2] = report.input[PS4_R2].value;
					}
					else output[PS4_R2] = r2;
				}
				//while (clock() - startTime < 34) {
					gcapi_Write(output);
				//}

			}

		}
	}

	// Free API resources and unload the library.
	gcdapi_Unload();
	FreeLibrary(hInsGPP);

	return 0;


}

