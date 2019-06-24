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

const float calibrationSquareDimension = 0.01905f;
const float arucoSquareDimension = 0.1016f;
const Size chessboardDimensions = Size(6, 9);

void createArucoMarkers() {
	Mat outputMarker;

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

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

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>) {

}

int main(int argv, char** argc) {
	createArucoMarkers();
}