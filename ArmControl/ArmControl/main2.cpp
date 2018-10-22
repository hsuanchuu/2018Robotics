#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include "opencv.hpp"
#include <vector>
#include <math.h>

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "4000"
#define PI 3.1415926

using namespace std;
using namespace cv;

vector<Moments> find_moments(Mat image, int thres){
	vector<vector<Point> > contours;
	findContours(image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	for(vector<vector<Point> >::iterator it = contours.begin(); it!=contours.end();){
		if(contourArea(*it) < thres)
			it = contours.erase(it);
		else
			++it;
	}

	vector<Moments> mu(contours.size());
	for(int i = 0; i < contours.size(); ++i){
		mu[i] = moments(contours[i], false);
	}

	return mu;
}

int sendCommand(char* sendbuf, SOCKET& ClientSocket)
{
	cout << "Sending Command: " << sendbuf << endl;
	int SendResult = 0;
	int ReceiveResult = 0;
	char recvbuf[512];
	int counter = 0;
	int RoundThreshold = 1000;

	//Send Command to Robot Arm
	SendResult = send(ClientSocket, sendbuf, strlen(sendbuf)+1, 0 );

    if (SendResult == SOCKET_ERROR)
	{
		cout<< "send failed with error: " << WSAGetLastError() << endl;
		closesocket(ClientSocket);
		WSACleanup();
		return 1;
	}

	Sleep(100);

	do
	{
		ReceiveResult = recv(ClientSocket, recvbuf, 512, 0);
		counter++;
	}while(ReceiveResult == 0 && counter < RoundThreshold);

	if(counter > RoundThreshold)
	{
		cout << "Respond Time Out" << endl;
		return 1;
	}
	else
	{
		if(!strcmp(recvbuf,"ERR"))
		{
			cout << "Invalid Command" << endl;
			return 1;
		}
	}

	return 0;
}

void getcoor(int*, int*, float, float);
int __cdecl main(void) 
{
    WSADATA wsaData;
    int iResult;

    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKET ClientSocket = INVALID_SOCKET;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

    int recvbuflen = DEFAULT_BUFLEN;
    
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }
	//cout << "hi" << endl;
    // No longer need server socket
    closesocket(ListenSocket);
	
	//========== Add your code below ==========//
	
	char setp2pspeed[] = "SETPTPSPEED 20";
	sendCommand(setp2pspeed, ClientSocket);
	char setlinespeed[] = "SETLINESPEED 20";
	sendCommand(setlinespeed, ClientSocket);
	char command[] = "GOHOME";
	sendCommand(command, ClientSocket);
	Sleep(5000);
	// 1. Read the camera frames and open a window to show it.

	

	VideoCapture cap(0+CV_CAP_VFW);

    if( !cap.isOpened() )
    {
        printf("\nCan not open camera or video file\n");
        system("pause");
        return -1;
    }

    // Capture a single frame
	Mat tmp_frame;
    cap >> tmp_frame;
    if(tmp_frame.empty())
    {
        printf("can not read data from the video source\n");
        system("pause");
        return -1;
    }

    // Build two display windows
    //namedWindow("video_cap", 1);
	//imshow("video_cap",tmp_frame);
	//waitKey(0.1);
    // the camera will be deinitialized automatically in VideoCapture destructor

	cvtColor(tmp_frame, tmp_frame, cv::COLOR_BGR2GRAY);

	Mat WorkImage = tmp_frame.clone();
	GaussianBlur(WorkImage, WorkImage, Size(3,3), 0, 0);
	threshold(WorkImage, WorkImage, 128, 255, THRESH_BINARY);

	erode(WorkImage, WorkImage, Mat());
	dilate(WorkImage, WorkImage, Mat());

	Mat ComImage = WorkImage.clone();

	vector<Moments> mu;
	int thres = 500;
	mu = find_moments(ComImage, thres);

	vector<Point2f> mc(mu.size());
	for(int i = 0; i < mu.size(); ++i){
		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}
	vector<double> phi(mu.size());
	for(int i = 0; i < mu.size(); ++i){
		phi[i] = atan2(2*mu[i].mu11, (mu[i].mu20 - mu[i].mu02)) / 2;
	}

	Mat OutImage;
	cvtColor(tmp_frame, OutImage, cv::COLOR_GRAY2BGR);
	vector<float> Cx;
	vector<float> Cy;
	vector<float> Theta;
	for(int i = 0; i < mu.size(); ++i){
		//cout<<int(mc[i].y)<<" "<<int(mc[i].x)<<" "<<90-int(phi[i] * 180./PI)<<endl;
		float cx = mc[i].x;
		float cy = mc[i].y;
		float theta = phi[i];
		int height = tmp_frame.rows;
		int r = 10;

		Cx.push_back(cx);
		Cy.push_back(cy);
		Theta.push_back(90-int(phi[i] * 180./PI));

		Point pt1(cx-cy / tan(theta), 0);
		Point pt2(cx + (height-cy) / tan(theta), height);
		Point pt3 = Point(cx-r, cy-r*tan(theta+PI/2));
		Point pt4 = Point(cx+r, cy+r*tan(theta+PI/2));

		line(OutImage, pt1, pt2, Scalar(0,0,255), 1, 8, 0);
		line(OutImage, pt3, pt4, Scalar(0,0,255), 1, 8, 0);

		//namedWindow("out", 1);
		//imshow("out", OutImage);
		//waitKey(10);
	}
	
	// 2. Segment the object(s) and calculate the centroid(s) and principle angle(s).

	// 3. Use prespective transform to calculate the desired pose of the arm.

	// 4. Move the arm to the grasping pose by sendCommand() function.
	// The following lines give an example of how to send a command.
	// You can find commends in "Robot Arm Manual.pdf"

	// 5. Control the gripper to grasp the object.
	// The following lines give an example of how to control the gripper.


	char em[100];
	// step 1: go straight down
	char rotateX[]="ROTATEX -45";
	sendCommand(rotateX,ClientSocket);
	
	int mx, my;
	getcoor(&mx, &my, Cy[0], Cx[0]);
	
	stringstream ss1,ss2,ss3;
	ss1<<mx;
	ss2<<my;
	int theta = ((int)(Theta[0]+90+180)%360-180);
	//cout << theta <<endl;
	ss3<<((int)(Theta[0]+90+180)%360-180);
	string str1="MOVP "+ss1.str()+" "+ss2.str()+" -90 " + ss3.str() + " # #";
	char target[100];
	strcpy(target,str1.c_str());
	char move1[100];
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);
	
	
	str1="MOVP # # -220 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);

	char closeGripper[] = "OUTPUT 48 ON";
	sendCommand(closeGripper, ClientSocket);

	str1="MOVP # # -150 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);

	str1="MOVP # # -220 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);
	
	
	Sleep(1000);
	char openGripper[] = "OUTPUT 48 OFF";
	sendCommand(openGripper, ClientSocket);

	str1="MOVP # # -150 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);

	

	
	getcoor(&mx, &my, Cy[1], Cx[1]);
	
	stringstream ss11,ss22,ss33;
	ss11<<mx;
	ss22<<my;
	theta = ((int)(Theta[1]+90+180)%360-180);
	//cout << theta <<endl;
	ss33<<((int)(Theta[1]+90+180)%360-180);
	//string str1="MOVP "+ss11.str()+" "+ss22.str()+" -150 " + ss33.str() + " # #";
	str1="MOVP "+ss11.str()+" "+ss22.str()+" -150 " + ss33.str() + " # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);

	str1="MOVP # # -220 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);

	//char closeGripper[] = "OUTPUT 48 ON";
	sendCommand(closeGripper, ClientSocket);

	str1="MOVP # # -150 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);

	sendCommand(target,ClientSocket);

	str1="MOVP # # -180 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);
	
	
	Sleep(1000);
	//char openGripper[] = "OUTPUT 48 OFF";
	sendCommand(openGripper, ClientSocket);

	str1="MOVP # # -150 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);
	





	getcoor(&mx, &my, Cy[2], Cx[2]);
	
	stringstream ss111,ss222,ss333;
	ss111<<mx;
	ss222<<my;
	theta = ((int)(Theta[2]+90+180)%360-180);
	//cout << theta <<endl;
	ss333<<((int)(Theta[2]+90+180)%360-180);
	//string str1="MOVP "+ss11.str()+" "+ss22.str()+" -150 " + ss33.str() + " # #";
	str1="MOVP "+ss111.str()+" "+ss222.str()+" -150 " + ss333.str() + " # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);

	str1="MOVP # # -220 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);

	//char closeGripper[] = "OUTPUT 48 ON";
	sendCommand(closeGripper, ClientSocket);

	str1="MOVP # # -90 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);

	sendCommand(target,ClientSocket);

	str1="MOVP # # -140 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);
	
	
	Sleep(1000);
	//char openGripper[] = "OUTPUT 48 OFF";
	sendCommand(openGripper, ClientSocket);

	str1="MOVP # # -90 # # #";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);






	/*str1="MOVP 0 200 -100 90 0 180";
	strcpy(move1,em);
	strcpy(move1,str1.c_str());
	sendCommand(move1,ClientSocket);
	*/
	//========== Add your code above ==========//
	
	sendCommand(command, ClientSocket);
	system("pause"); 
		
	// shutdown the connection since we're done
    iResult = shutdown(ClientSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        return 1;
    }

    // cleanup
    closesocket(ClientSocket);
    WSACleanup();

    return 0;
}

void getcoor(int* mx, int* my, float cx, float cy){
	//*my = (int)(1.1822*cx + 265.1);
	//*mx = (int)(-1.1595*cy + 291.04);
	*my = (int)(1.0448*cx + 287.61);
	*mx = (int)(-1.0125*cy + 245.49);
}
