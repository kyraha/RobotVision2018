package org.error3130.powerup;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.error3130.powerup.DetectLED;
import org.error3130.powerup.DetectLED.Chain;

public class Main {
	public static void main( String[] args )
	{
		System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
		System.out.println("Library loaded: "+ Core.NATIVE_LIBRARY_NAME);

		double imageSize;

		if(args.length > 0 ) {
			Mat image = Imgcodecs.imread(args[0]);
			if(image.empty()) {
				System.err.println("Could not import file "+ args[0]);
				System.err.println("Working dir: "+ System.getProperty("user.dir"));
				return;
			}
			imageSize = Math.sqrt(image.size().area());

			HighGui.imshow("test", image);
			HighGui.waitKey();

			Rect roiRect = new Rect(new Point(0, image.height()/5), new Size(image.width(), image.height()/3));

			DetectLED detector = new DetectLED()
					.withThresh(120)
					.withMinArea(imageSize/400)
					.withMaxArea(imageSize/16)
					.withMaxSegment(imageSize/5);

			detector.findLEDs(image, roiRect)
					.findSegments()
					.findChains();

			System.out.println("Lights detected: "+ detector.lights.size());
			System.out.println("Chains detected: "+ detector.chains.size());
			for(int i = 0; i < detector.lights.size(); i++) {
				Point center = detector.lights.get(i);
				Imgproc.circle(image, center , (int)(imageSize/70), new Scalar(255,0,255));
				Imgproc.putText(image, " "+i, center, 1, 1, new Scalar(0,0,255));
			}
			HighGui.imshow("test", image);
			HighGui.waitKey();


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
					System.out.println(sp.nodeIndex +" "+ sp.score());
				}
			}
			HighGui.imshow("test", image);
			HighGui.waitKey(0);

			roiRect = new Rect(new Point(0, 2*image.height()/3), new Size(image.width(), image.height()/3));

			detector.findLEDs(image, roiRect)
					.findSegments()
					.findChains();

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
					System.out.println(sp.nodeIndex +" "+ sp.score());
				}
			}
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
