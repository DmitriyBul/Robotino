#define _USE_MATH_DEFINES
#include "opencv2/highgui/highgui.hpp"

#include <cmath>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>


#include "rec/robotino/com/all.h"
#include "rec/core_lt/utils.h"
#include "rec/core_lt/Timer.h"
using namespace cv;
using namespace std;

//#include <opencv2\highgui\highgui.hpp>

using namespace std;


using namespace rec::robotino::com;

class MyCom : public Com
{
public:
	MyCom()
	{
	}

	void errorEvent(Error error, const char* errorString)
	{
		std::cerr << "Error: " << errorString << std::endl;
	}

	void connectedEvent()
	{
		std::cout << "Connected." << std::endl;
	}

	void connectionClosedEvent()
	{
		std::cout << "Connection closed." << std::endl;
	}

	void modeChangedEvent(bool isPassiveMode)
	{
		if (isPassiveMode)
		{
			std::cout << "Connected int passive mode." << std::endl;
		}
	}
};


MyCom com;
OmniDrive omniDrive;


void init(const std::string& hostname)
{
	// Initialize the actors

	// Connect
	std::cout << "Connecting..." << std::endl;
	com.setAddress(hostname.c_str());

	com.connect();

	std::cout << std::endl << "Connected" << std::endl;
}

void drive()
{
	while (com.isConnected()) /*com.isConnected() возвращает true,если Robotino имеет связь с компьютером */
	{
		omniDrive.setVelocity(100, 0, 0); //движение вперед 100 мм/с
		return;
	}
}

void destroy()
{
	com.disconnect();
}


int main(int argc, char** argv)
{
	VideoCapture cap;
	// open the default camera, use something different from 0 otherwise;
	// Check VideoCapture documentation.
	if (!cap.open(0))
		return 0;
	for (;;)
	{
		Mat frame;
		Mat greyMat;
		Mat binary;

		cap >> frame;
		if (frame.empty()) break; // end of video stream
		cvtColor(frame, greyMat, COLOR_BGR2GRAY);
		double thresh = 50;
		double maxValue = 155;
		threshold(greyMat, binary, thresh, maxValue, THRESH_BINARY);

		cv::Mat top_left
			= binary(cv::Range(0, binary.rows / 2 - 1), cv::Range(0, binary.cols * 3 / 7 - 1));
		cv::Mat top_right
			= binary(cv::Range(0, binary.rows / 2 - 1), cv::Range(binary.cols * 4 / 7, binary.cols - 1));
		cv::Mat top
			= binary(cv::Range(0, binary.rows / 2 - 1), cv::Range(binary.cols * 3 / 7, binary.cols * 4 / 7 - 1));
		cv::Mat bottom_left
			= binary(cv::Range(binary.rows / 2, binary.rows - 1), cv::Range(0, binary.cols * 3 / 7 - 1));
		cv::Mat bottom_right
			= binary(cv::Range(binary.rows / 2, binary.rows - 1), cv::Range(binary.cols * 4 / 7, binary.cols - 1));
		cv::Mat bottom
			= binary(cv::Range(binary.rows / 2, binary.rows - 1), cv::Range(binary.cols * 3 / 7, binary.cols * 4 / 7 - 1));

		int pixels_tl = top_left.rows * top_left.cols;
		int zeropixels_tl = pixels_tl - countNonZero(top_left);
		int pixels_tr = top_right.rows * top_right.cols;
		int zeropixels_tr = pixels_tr - countNonZero(top_right);
		int pixels_t = top.rows * top.cols;
		int zeropixels_t = pixels_t - countNonZero(top);

		int pixels_bl = bottom_left.rows * bottom_left.cols;
		int zeropixels_bl = pixels_bl - countNonZero(bottom_left);
		int pixels_br = bottom_right.rows * bottom_right.cols;
		int zeropixels_br = pixels_br - countNonZero(bottom_right);
		int pixels_b = bottom.rows * bottom.cols;
		int zeropixels_b = pixels_b - countNonZero(bottom);
		

		//cout << "The number of pixels that are zero is " << ZeroPixels_tl << endl;
		//threshold(greyMat, output, thresh, maxValue, THRESH_BINARY);

		/*
		cv::imshow("top_left", top_left);
		cv::imshow("top_right", top_right);
		cv::imshow("top", top);
		cv::imshow("bottom_left", bottom_left);
		cv::imshow("bottom_right", bottom_right);
		cv::imshow("bottom", bottom);
		*/
		cv::imshow("frame", binary);
		//cv::imshow("output", output);
		//cv::waitKey(0);
		if (waitKey(10) == 27) break; // stop capturing by pressing ESC 
	}
	std::string hostname = "172.26.1.1";
	if (argc > 1)
	{
		hostname = argv[1];
	}

	try
	{
		init(hostname);

		drive();

		
		
		//forward();
		destroy();
	}
	catch (const rec::robotino::com::ComException& e)
	{
		std::cerr << "Com Error: " << e.what() << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
	}
	catch (...)
	{
		std::cerr << "Unknow Error" << std::endl;
	}
	
	//std::cout << "Press any key to exit..." << std::endl;
	//rec::core_lt::waitForKey();
}
