//============================================================================
// Name        : powerup-led.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#define _USE_MATH_DEFINES
#include "opencv2/opencv.hpp"
// #include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/highgui/highgui.hpp"

#include <iostream>

using namespace std;


// Coordinates in the real world in inches
static const std::vector<cv::Point3f> realPoints {
	{ 70,-15,140}, {-70, -3,140}, // switch
	{ 85,-60,300}, {-59,-60,300}  // Scale
};

static cv::Mat intrinsic, distortion;

bool readIntrinsics(const char *filename)
{
	cv::FileStorage fs( filename, cv::FileStorage::READ );
	if( !fs.isOpened() )
	{
		std::cerr << " Error: Couldn't open intrinsic parameters file "
				<< filename << std::endl;
		return false;
	}
	fs["camera_matrix"] >> intrinsic;
	fs["distortion_coefficients"] >> distortion;
	if( intrinsic.empty() || distortion.empty() )
	{
		std::cerr << " Error: Couldn't load intrinsic parameters from "
				<< filename << std::endl;
		return false;
	}
	fs.release();
	return true;
}

cv::Point2f getRobotCoordinates(std::vector<cv::Point2f> imagePoints)
{
	cv::Vec3d rvec, tvec, robot_loc;
	cv::Matx33d rmat, cmat;
	cv::solvePnP(
			realPoints,       // 3-d points in object coordinate
			imagePoints,        // 2-d points in image coordinates
			intrinsic,           // Our camera matrix
			distortion,
			rvec,                // Output rotation *vector*.
			tvec                 // Output translation vector.
	);
	cout << "Translation vector: " << tvec << std::endl;
	cout << "Rotation vector: " << rvec << std::endl;
	cv::Rodrigues(rvec, rmat);
	robot_loc = rmat.t() * -tvec;
	cout << "Camera location: " << robot_loc << std::endl;
	return cv::Point2d(robot_loc(0), robot_loc(2));
}

void process(cv::Mat image)
{
	// Coordinates in the picture in pixels
	// Let's pretend we detected and identified the features by color filtering and whatsnot
	cv::Point sw_leftou = cv::Point( 451, 698);	// switch left plate outer corner
	cv::Point sw_righto = cv::Point(1135, 394);	// switch right plate outer corner
	cv::Point sc_righto = cv::Point( 612,  95);	// scale right plate outer corner
	cv::Point sc_leftin = cv::Point(  34, 122);	// scale left plate inner corner

	// Pack the image points in a vector as well making sure they're floating points too
	std::vector<cv::Point2f> imagePoints;
	imagePoints.push_back(sw_righto);
	imagePoints.push_back(sw_leftou);
	imagePoints.push_back(sc_righto);
	imagePoints.push_back(sc_leftin);

	// Draw circles to see if we get the points of interest right
	for(auto it: imagePoints) cv::circle(image, it, 12, cv::Scalar(0,255,155));

	cv::Point2f myLocation = getRobotCoordinates(imagePoints);

	cout << "Robot location: " << myLocation << std::endl;
}

int main(int argc, char** argv) {
	if(argc != 3) {
		std::cerr << "Format: "<< argv[0] <<" image.file intrinsics.yml" << std::endl;
		return 2;
	}
	cv::String filename = cv::String(argv[1]);
	const char* intrinsic_file = argv[2];

	if(!readIntrinsics(intrinsic_file)) {
		std::cerr << "Could not read intrinsics file" << std::endl;
		return 1;
	}

    cv::Mat image;
    image = cv::imread(filename);
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    process(image);
    cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    cv::waitKey(0);

	return 0;
}
