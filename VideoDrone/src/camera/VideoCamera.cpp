/* FileName: VideoCamera.h
Created by Rakshit Allamraju
Date 3 July 2017

Thread to start video server and downlink camera data to the app
*/

#include "../../include/camera/PracticalSocket.h"
#include <iostream>              
#include <cstdlib>                

using namespace std;

//#include <raspicam/raspicam_cv.h>
#include "opencv2/opencv.hpp"
using namespace cv;
#include "../../include/camera/config.h"

#define CAMERAPORT "12000"

void* RunCamera(){

    //raspicam::RaspiCam_Cv Camera;
    //Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    //cout<<"Opening Camera..."<<endl;
    //if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}

    unsigned short clientPort = atoi(CAMERAPORT);//Loaded from DroneConfig.h
    string sourceAddress;
    unsigned short sourcePort;
    int recvMsg;

    try {
        UDPSocket sock(clientPort);
        int jpegqual =  ENCODE_QUALITY; // Compression Parameter

        Mat frame, send;
        vector < uchar > encoded;
        VideoCapture cap(0); // Grab the camera

        if (!cap.isOpened()) {
            cerr << "OpenCV Failed to open camera";
            exit(1);
        }

	char tempBuff[1024];
	recvMsg = sock.recvFrom(tempBuff, 1024, sourceAddress, sourcePort);

        clock_t last_cycle = clock();
        while (1) {

            //Camera.grab();
            //Camera.retrieve (frame);
	    cap >> frame;

            if(frame.size().width==0)continue;//simple integrity check; skip erroneous data...
            resize(frame, send, Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, INTER_LINEAR);
            vector < int > compression_params;
            compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            compression_params.push_back(jpegqual);

            imencode(".jpg", send, encoded, compression_params);
            //imshow("send", send);
            int total_pack = 1 + (encoded.size() - 1) / PACK_SIZE;

            int ibuf[1];
            ibuf[0] = total_pack;
            sock.sendTo(ibuf, sizeof(int), sourceAddress, sourcePort);

            for (int i = 0; i < total_pack; i++)
                sock.sendTo( & encoded[i * PACK_SIZE], PACK_SIZE, sourceAddress, sourcePort);

            waitKey(FRAME_INTERVAL);

            //clock_t next_cycle = clock();
            //double duration = (next_cycle - last_cycle) / (double) CLOCKS_PER_SEC;
            //cout << "\teffective FPS:" << (1 / duration) << " \tkbps:" << (PACK_SIZE * total_pack / duration / 1024 * 8) << endl;

            //cout << next_cycle - last_cycle;
            //last_cycle = next_cycle;
        }
        // Destructor closes the socket
	//Camera.release();

    } catch (SocketException & e) {
        cerr << e.what() << endl;
        exit(1);
    }

    return 0;


}

