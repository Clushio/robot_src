#include "stdafx.h"  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    cv::VideoCapture vcap;
    cv::Mat image;

    const std::string videoStreamAddress = "http://192.168.1.142:81/videostream.cgi?user=admin&pwd=888888&.mjpg";
    /*Address e.g. "http://IP:port/videostream.cgi?user=admin&pwd=******&.mjpg" */

    //open the video stream and make sure it's opened
    if (!vcap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    cv::namedWindow("Output Window");
    while (1) {
        for (;;) {
            if (!vcap.read(image)) {
                std::cout << "No frame" << std::endl;
                cv::waitKey();
            }
            cv::imshow("Output Window", image);
            if (cv::waitKey(1) >= 0) break;
        }
    }
    cvWaitKey(0);
    vcap.~VideoCapture();  
}