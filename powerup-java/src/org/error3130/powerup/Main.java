package org.error3130.powerup;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.List;

import org.error3130.powerup.DetectLED;
import org.error3130.powerup.DetectLED.Chain;

public class Main {
	static{
		System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
		System.out.println("Library loaded: "+ Core.NATIVE_LIBRARY_NAME);
	}

	static double camera_matrix[] = { 1.1312010798594224e+03, 0., 6.2056706058995383e+02, 0.,
	           1.1312010798594224e+03, 3.3312414963664799e+02, 0., 0., 1. };
	static double camera_dist[] = { 1.2499165734503319e-01, -1.1009971037701471e+00,
	                                          5.3431557229252234e-04, -3.2697740842547215e-04,
	                                          2.0261366290992946e+00 };
	static MatOfPoint3f objectPoints;
	static Mat cameraMatrix;
	static MatOfDouble distCoeffs;


	public static void processFrame(Mat image) {
		double imageSize = Math.sqrt(image.size().area());

		MatOfPoint2f imagePoints = new MatOfPoint2f();

		Rect roiRect = new Rect(new Point(0, image.height()/5), new Size(image.width(), image.height()/3));

		DetectLED detector = new DetectLED()
				.withThresh(60)
				.withMinArea(imageSize/400)
				.withMaxArea(imageSize/10)
				.withMaxSegment(imageSize/5);

		detector.findLEDs(image, roiRect)
				.findSegments()
				.findChains()
				.getCorners();

		MatOfPoint2f twoCorners = detector.getCorners();
		if(twoCorners == null) return;
		imagePoints.push_back(twoCorners);

		for(int i = 0; i < detector.lights.size(); i++) {
			Point center = detector.lights.get(i);
			Imgproc.circle(image, center , (int)(imageSize/70), new Scalar(255,0,255));
			Imgproc.putText(image, " "+i, center, 1, 1, new Scalar(0,0,255));
		}

		if(detector.chains.size() > 0) {
			Chain sp = detector.chains.get(0);
			if(sp.nodes.size() > 1) {
				int i = 0;
				Point2 pointA = new Point2(0,0);
				for(Point2 pointB: sp.nodes) {
					if(i > 0) {
						Imgproc.line(image, pointA, pointB, new Scalar(0, 255, 255));
					}
					pointA = pointB;
					i++;
				}
			}
		}

		roiRect = new Rect(new Point(0, 2*image.height()/3), new Size(image.width(), image.height()/3));

		detector.findLEDs(image, roiRect)
				.findSegments()
				.findChains();

		twoCorners = detector.getCorners();
		if(twoCorners == null) return;
		imagePoints.push_back(twoCorners);

		for(int i = 0; i < detector.lights.size(); i++) {
			Point center = detector.lights.get(i);
			Imgproc.circle(image, center , (int)(imageSize/70), new Scalar(255,0,255));
			Imgproc.putText(image, " "+i, center, 1, 1, new Scalar(0,0,255));
		}
		if(detector.chains.size() > 0) {
			Chain sp = detector.chains.get(0);
			if(sp.nodes.size() > 1) {
				int i = 0;
				Point2 pointA = new Point2(0,0);
				for(Point2 pointB: sp.nodes) {
					if(i > 0) {
						Imgproc.line(image, pointA, pointB, new Scalar(0, 255, 255));
					}
					pointA = pointB;
					i++;
				}
			}
		}
		List<Point> corners = imagePoints.toList();
		for(Point corner: corners) {
			Point a = new Point2(-imageSize/40, -imageSize/40).plus(corner);
			Point b = new Point2( imageSize/40,  imageSize/40).plus(corner);
			Imgproc.rectangle(image, a, b, new Scalar(0,255,0));
		}

		Mat rvec = new Mat();
		Mat tvec = new Mat();
		Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
		String text = String.format("Tvec %6.1g %6.1g %6.1g",
			tvec.get(0,0)[0],
			tvec.get(1,0)[0],
			tvec.get(2,0)[0]);
		Imgproc.putText(image, text, new Point(10,10), 1, 1, new Scalar(0,255,0));
		text = String.format("Rvec %6.1g %6.1g %6.1g",
				rvec.get(0,0)[0],
				rvec.get(1,0)[0],
				rvec.get(2,0)[0]);
		Imgproc.putText(image, text, new Point(10,40), 1, 1, new Scalar(0,255,0));
	}

	public static void main( String[] args )
	{
		objectPoints = new MatOfPoint3f();
		List<Point3> objectReal = new ArrayList<Point3>();
		objectReal.add(new Point3(55.09, -57, 300));
		objectReal.add(new Point3(89.44, -57, 300));
		objectReal.add(new Point3(36.91, -8, 146));
		objectReal.add(new Point3(71.02, -10, 146));
		objectPoints.fromList(objectReal);

		distCoeffs = new MatOfDouble(camera_dist);
		cameraMatrix = new Mat(3, 3, CvType.CV_32F);
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++) {
				cameraMatrix.put(i, j, camera_matrix[3*i+j]);
			}
		
		if(args.length > 0 ) {
			Mat image = Imgcodecs.imread(args[0]);
			if(image.empty()) {
				System.err.println("Could not import file "+ args[0]);
				System.err.println("Working dir: "+ System.getProperty("user.dir"));
				return;
			}
			HighGui.imshow("test", image);
			HighGui.waitKey();

			processFrame(image);

			HighGui.imshow("test", image);
			HighGui.waitKey(0);
		}
		else {
			System.out.println("File name is missing. Trying to feed from camera");

			Mat frame = new Mat();
			VideoCapture cap = new VideoCapture();
			cap.open(0);

			if(cap.isOpened()) {
				do {
					cap.read(frame);
					processFrame(frame);
					HighGui.imshow("test", frame);
				} while (HighGui.waitKey(10) < 0);
			}
			else {
				System.err.println("Cannot open a video stream");
			}
		}

		HighGui.destroyAllWindows();
		System.out.println("All done");
		System.exit(0);
	}
}
