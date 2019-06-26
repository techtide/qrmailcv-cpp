#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/aruco.hpp"

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

// These measurements are measured in meters.
// Remember that the measurements should be very accurate; this is one of the most important things you can do.
// Also, while the recommended amount of calibration photos is 10, you need much more, like 50.

const float calibrationSquareDimension = 0.01905f;
const float arucoSquareDimension = 0.1016f;
const Size chessboardDimensions = Size(6, 9);

void createArucoMarkers() {
	Mat outputMarker;

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);

	for (int i = 0; i < 50; i++) {
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".jpg";
		imwrite(convert.str(), outputMarker);
		
	}

}

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) {
	for (int j = 0; j < boardSize.height; j++) {
		for (int k = 0; k < boardSize.width; k++) {
			corners.push_back(Point3f(j * squareEdgeLength, j * squareEdgeLength, 0));
		}
	}
}

/*void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>) {
	
}*/
/*
int main(int argv, char** argc) {
	createArucoMarkers();
}
*/