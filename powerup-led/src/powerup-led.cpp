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


static cv::Mat intrinsic, distortion;

std::string date_now()
{
	std::time_t result = std::time(nullptr);
	std::string str(std::asctime(std::localtime(&result)));
	// trim trailing endl
	size_t endpos = str.find_last_not_of(" \t\n");
	if( str.npos != endpos )
	{
	    str = str.substr( 0, endpos+1 );
	}
	return str + " ";
}

bool readIntrinsics(const char *filename)
{
	cv::FileStorage fs( filename, cv::FileStorage::READ );
	if( !fs.isOpened() )
	{
		std::cerr << date_now() << " Error: Couldn't open intrinsic parameters file "
				<< filename << std::endl;
		return false;
	}
	fs["camera_matrix"] >> intrinsic;
	fs["distortion_coefficients"] >> distortion;
	if( intrinsic.empty() || distortion.empty() )
	{
		std::cerr << date_now() << " Error: Couldn't load intrinsic parameters from "
				<< filename << std::endl;
		return false;
	}
	fs.release();
	return true;
}

void process(cv::Mat image)
{
	// Coordinates in the real world in inches
	static const std::vector<cv::Point3f> realPoints {
		{-52, -8,140}, {-68, -8,140}, // switch
		{ 85,-60,300}, {-59,-60,300}  // Scale
	};
	// Coordinates in the picture in pixels
	cv::Point vt_center = cv::Point(794/1.3125,880/1.3125-40);	// vision target center
	cv::Point sw_leftou = cv::Point(591/1.3125,969/1.3125-40);	// switch left plate outer corner
	cv::Point sc_righto = cv::Point(804/1.3125,177/1.3125-40);	// scale right plate outer corner
	cv::Point sc_leftin = cv::Point( 66/1.3125,212/1.3125-40);	// scale left plate inner corner
	// Draw circles to see if we get the points of interest right
	cv::circle(image, vt_center, 20, cv::Scalar(0,255,0));
	cv::circle(image, sw_leftou, 20, cv::Scalar(0,255,0));
	cv::circle(image, sc_leftin, 20, cv::Scalar(0,255,0));
	cv::circle(image, sc_righto, 20, cv::Scalar(0,255,0));
	// Pack the image points in a vector as well making sure they're floating points too
	std::vector<cv::Point2f> imagePoints;
	imagePoints.push_back(vt_center);
	imagePoints.push_back(sw_leftou);
	imagePoints.push_back(sc_righto);
	imagePoints.push_back(sc_leftin);
	cv::Vec3d rvec, tvec, robot_loc, target_loc;
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
}

int main(int argc, char** argv) {
	if(argc != 3) {
		std::cerr << "Format: this-program task intrinsics.yml" << std::endl
				<< "Tasks: Peg or Boiler" << std::endl;
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
