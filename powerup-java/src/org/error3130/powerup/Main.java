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

import java.util.ArrayList;
import java.util.List;

import org.error3130.powerup.DetectLED;
import org.error3130.powerup.DetectLED.Chain;

public class Main {
	static double camera_matrix[] = { 1.1312010798594224e+03, 0., 6.2056706058995383e+02, 0.,
	           1.1312010798594224e+03, 3.3312414963664799e+02, 0., 0., 1. };
	static double camera_dist[] = { 1.2499165734503319e-01, -1.1009971037701471e+00,
	                                          5.3431557229252234e-04, -3.2697740842547215e-04,
	                                          2.0261366290992946e+00 };
	public static void main( String[] args )
	{
		System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
		System.out.println("Library loaded: "+ Core.NATIVE_LIBRARY_NAME);

		List<Point3> objectReal = new ArrayList<Point3>();
		objectReal.add(new Point3(55.09, -57, 300));
		objectReal.add(new Point3(89.44, -57, 300));
		objectReal.add(new Point3(36.91, -8, 146));
		objectReal.add(new Point3(71.02, -10, 146));
		MatOfPoint3f objectPoints = new MatOfPoint3f();
		objectPoints.fromList(objectReal);

		Mat cameraMatrix = new Mat(3, 3, CvType.CV_32F);
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++) {
				cameraMatrix.put(i, j, camera_matrix[3*i+j]);
			}
		MatOfDouble distCoeffs = new MatOfDouble(camera_dist);
		
		double imageSize;

		if(args.length > 0 ) {
			Mat image = Imgcodecs.imread(args[0]);
			if(image.empty()) {
				System.err.println("Could not import file "+ args[0]);
				System.err.println("Working dir: "+ System.getProperty("user.dir"));
				return;
			}
			HighGui.imshow("test", image);
			HighGui.waitKey();

			imageSize = Math.sqrt(image.size().area());

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

			imagePoints.push_back(detector.getCorners());

			System.out.println("Lights detected: "+ detector.lights.size());
			System.out.println("Chains detected: "+ detector.chains.size());
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
					System.out.println(sp.nodeIndex +" "+ sp.score);
				}
			}

			roiRect = new Rect(new Point(0, 2*image.height()/3), new Size(image.width(), image.height()/3));

			detector.findLEDs(image, roiRect)
					.findSegments()
					.findChains();

			imagePoints.push_back(detector.getCorners());

			System.out.println("Lights detected: "+ detector.lights.size());
			System.out.println("Chains detected: "+ detector.chains.size());
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
					System.out.println(sp.nodeIndex +" "+ sp.score);
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
			System.out.println("tvec.x: " + tvec.get(0,0)[0]);
			System.out.println("tvec.y: " + tvec.get(1,0)[0]);
			System.out.println("tvec.z: " + tvec.get(2,0)[0]);
			System.out.println("rvec.x: " + rvec.get(0,0)[0]);
			System.out.println("rvec.y: " + rvec.get(1,0)[0]);
			System.out.println("rvec.z: " + rvec.get(2,0)[0]);
			HighGui.imshow("test", image);
			HighGui.waitKey(0);

			HighGui.destroyAllWindows();
		}
		else {
			System.err.println("File name required");
			return;
		}
		System.out.println("All done");
		System.exit(0);
	}
}
