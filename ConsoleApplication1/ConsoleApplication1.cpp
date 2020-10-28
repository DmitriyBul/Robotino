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
int view_line()
{
	VideoCapture cap;
	// open the default camera, use something different from 0 otherwise;
	// Check VideoCapture documentation.
	if (!cap.open(1))
		return 10;

	double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count
	cap.set(CV_CAP_PROP_POS_FRAMES, count - 1); //Set index to last frame
	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE);
	Mat frame;
	Mat greyMat;
	Mat binary;

	bool success = cap.read(frame);
	if (!success) {
		cout << "Cannot read  frame " << endl;
	}
	//imshow("MyVideo", frame);
	imwrite("image.jpg", frame);
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

	if (zeropixels_b > 10 && zeropixels_t > 10)
		return 1;
	
}
void drive()
{
	int info;
	rec::core_lt::Timer timer; //создание объекта класса Timer
	while (com.isConnected()) /*com.isConnected() возвращает true,если Robotino имеет связь с компьютером */
	{
		while (1) {


			VideoCapture cap;
			// open the default camera, use something different from 0 otherwise;
			// Check VideoCapture documentation.
			if (!cap.open(1))
				return;

			double count = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count
			cap.set(CV_CAP_PROP_POS_FRAMES, count - 1); //Set index to last frame
			namedWindow("MyVideo", CV_WINDOW_AUTOSIZE);
			Mat frame;
			Mat greyMat;
			Mat binary;

			bool success = cap.read(frame);
			if (!success) {
				cout << "Cannot read  frame " << endl;
			}
			//imshow("MyVideo", frame);
			imwrite("image.jpg", frame);
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

			if(zeropixels_b > 80 && zeropixels_t > 80){
				omniDrive.setVelocity(100, 0, 0);
			}
			if (zeropixels_tr > 200 && zeropixels_tr > zeropixels_t) {
				omniDrive.setVelocity(0, 20, -10);
			}
			if (zeropixels_tl > 200 && zeropixels_tl > zeropixels_t) {
				omniDrive.setVelocity(0, -20, 10);
			}
			/*
			else {
				timer.start();
				while(timer.msecsElapsed() < 2000){ 
					omniDrive.setVelocity(100, 0, 20); 
					
				}
				omniDrive.setVelocity(0, 0, 0);
			}
			*/
			/*
			if (((zeropixels_t > zeropixels_tr) && (zeropixels_t > zeropixels_tl)) && ((zeropixels_b > zeropixels_bl) && (zeropixels_b > zeropixels_br)) && zeropixels_b > 20 && zeropixels_t > 20) {
				omniDrive.setVelocity(100, 0, 0);
			}
			if((zeropixels_tl > zeropixels_tr || zeropixels_bl > zeropixels_br) && (zeropixels_tl > 15 || zeropixels_bl > 15)) {
				timer.start();
				while (timer.msecsElapsed() < 300) {
					omniDrive.setVelocity(100, 0, 0); //движение вперед 100 мм/с
				}
				timer.reset();
				while (zeropixels_tl > 5) {
					omniDrive.setVelocity(0, 0, 50);
				}

			}*/
			//omniDrive.setVelocity(100, 0, 0); //движение вперед 100 мм/с
		}
		return;
		
	}
}

void destroy()
{
	com.disconnect();
}


int main(int argc, char** argv)
{
		
	
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
