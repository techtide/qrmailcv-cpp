#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/aruco/charuco.hpp"

#include "math.h"

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

// Debugging methods
string type2str(int type);

// For the desktop camera I'm using - the Logitech HD Webcam C270.
/*double hFieldOfView = 60; // Degrees
double vFieldOfView = 46.826; // Degrees*/
double hFieldOfView = 60;
double vFieldOfView = 46.826;
double focalLength = 4.000; // Milimeters

// TO-DO: FIND THIS AND POPULATE ObjectPoints correctly as discussed in IRC
// Measure this carefully as it corresponds to how it is shown on the screen, use it for objectPoints.
double arucoMarkerLength = 100; // Milimeters

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();

void process(Mat& image, vector<vector<Point2f>> imagePoints) {
	// This should be used with a single Aruco for now, as it assumes that the object space points are (0, 0, 0).
	// KEEP FOR FINAL: std::cout << "passing conditions" << std::endl;
	// Now do the PnP and get the tvec and rvec, which can then be used for doing AR.
	cv::Mat rvec, tvec;
	// (0,0,0) for all of them won't work! Object points could be for example (0,0,0) (0,length,0) (length, 0, 0) (length, length, 0, 0)
	// ORDER MATTERS. This should correspond to the points in imageMatrix!
	// the original:
	// vector<Point3f> objectPoints(4, Point3f(0));
	// the new (not tried yet):
	vector<Point3f> objectPoints = { Point3f(0,0,0), Point3f(arucoMarkerLength, 0, 0), 
									 Point3f(arucoMarkerLength, arucoMarkerLength, 0), Point3f(0, arucoMarkerLength, 0) };
	// Temporarily estimate the camera matrix without doing any manual camera calibration (remember to adjust the FoV and focal length).
	double width = image.size().width;
	double height = image.size().height;
	try {
		//Matx33f needs to be of type CV_64F.
		/*cv::Matx33d cameraMatrix((float)(width / 2 * tan(hFieldOfView * 3.14159 / 180)), (float)0, (float)width / 2,
			(float)0, (float)height / (float)(2 * tan(vFieldOfView * 3.14159 / 180)), (float)height / 2,
			(float)0, (float)0, (float)1);*/

			/*vector<vector<double>> vec = { {(double)(width / 2 * tan(hFieldOfView * 3.14159 / 180)), (double)0, (double)width / 2},
			{(double)0, (double)height / (double)(2 * tan(vFieldOfView * 3.14159 / 180)), (double)height / 2},
			{(double)0, (double)0, (double)1}};
			cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F);*/

		double fx = (double)(width / 2 * tan(hFieldOfView * 3.14159 / 180));
		double cx = (double)width / 2;
		double fy = (double)height / (double)(2 * tan(vFieldOfView * 3.14159 / 180));
		double cy = (double)height / 2;

		//Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
		// THESE CAMERA MATRIX CALCULATIONS WERE WRONG ^^^ - THE ONE BELOW IS TAKEN FROM A YAML FILE I FOUND ON INTERNET
		// EVENTUALLY MAKE CALIBRATION GET CAMERA MATRIX BY ITSELF BUT FIRST UNDERSTAND PROPERLY THE CAMERA MATRIX
		Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 5.2036144200036904e+02, 0, 3.0255876920712751e+02, 0,
			5.1834337983691967e+02, 2.4278187493920279e+02, 0, 0, 1);
		// std::cout << cameraMatrix.channels() << std::endl;
		// std::cout << "Type " << type2str(cameraMatrix.type()) << std::endl;

		//cv::Mat distCoeffs = Mat::zeros(8, 1, cv::DataType<double>::type);
		Mat distCoeffs = (cv::Mat_<double>(5, 1) << 1.0303738535136008e-01, -2.7013831868935040e-01, -2.5505644429536711e-03, -6.9928818760003142e-03, 1.6547444868453295e-01);

		//int npoints = std::max(objectPoints.checkVector(3, CV_32F), objectPoints.checkVector(3, CV_64F));
		//std::cout << "npoints camera matrix " << npoints << std::endl;


		vector<Point2d> imagePointsDouble = { (Point2d) (imagePoints.at(0).at(0)), (Point2d)(imagePoints.at(0).at(1)),
											  (Point2d)(imagePoints.at(0).at(2)), (Point2d)(imagePoints.at(0).at(3))};
		//cv::Mat imgPtsMat = cv::Mat(imagePointsDouble, CV_64F);
		//std:cout << "ipoints image matrix " << std::max(imgPtsMat.checkVector(2, CV_32F), imgPtsMat.checkVector(2, CV_64F)) << std::endl;
		//CV_Assert(npoints == std::max(imgPtsMat.checkVector(2, CV_32F), imgPtsMat.checkVector(2, CV_64F)));

		if (!imagePoints.empty()) {
			// off true once fixed
			cv::solvePnP(objectPoints, imagePoints.at(0), cameraMatrix, distCoeffs, rvec, tvec, false);
			cv::drawFrameAxes(image, cameraMatrix, distCoeffs, rvec, tvec, 100);
		}
		else {
			std::cout << "No arucos found, so PnP cannot be solved" << std::endl;
		}
	}
	catch (cv::Exception& e) {
		cerr << e.what() << endl;
	}
}

// This is from SO Community.
string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

int main(int argv, char* argc) {
	cv::VideoCapture capture;
	if (!capture.open(0)) {
		std::cout << "Camera not enabled" << std::endl;
		return 0;
	}
	Mat image;
	for (;;) {
		capture >> image;
		if (image.empty()) {
			break;
		}
		vector<vector<Point2f>> corners;
		vector<int> ids;
		vector<vector<Point2f>> rejectedImgPoints;
		
		Mat grayscaledImage;
		cv::cvtColor(image, grayscaledImage , cv::COLOR_BGR2GRAY);
		
		cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejectedImgPoints);
		cv::aruco::drawDetectedMarkers(image, corners, ids, cv::Scalar(0, 255, 0));

		if (cv::waitKey(10) == 97) {
			// when a key is pressed
			// print the corners
			std::cout << corners.at(0) << std::endl;
		}

		if (!ids.empty()) {
			// probably need to for loop this for each aruco (given by the corners)
			process(image, corners);
		}

		cv::imshow("live video capture", image);
		if (cv::waitKey(10) == 27) {
			break;
		}
	}
	return 0;
}

