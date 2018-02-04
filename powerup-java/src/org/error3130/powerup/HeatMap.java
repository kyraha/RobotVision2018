/**
 * 
 */
package org.error3130.powerup;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;

/**
 * @author mkyraha
 *
 */
public class HeatMap {
	public static void main( String[] args ) {
		System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
		System.out.println("Library loaded: "+ Core.NATIVE_LIBRARY_NAME);
		Mat image = Mat.zeros(480, 640, CvType.CV_8UC3);
		DetectLED detector = new DetectLED();
		detector.lights.add(new Point(100,200));
		detector.lights.add(new Point(150,200));
		detector.lights.add(new Point(200,200));
		detector.lights.add(new Point(250,205));
		detector.lights.add(new Point(300,200));
		detector.chains.add(detector.new Chain(detector.new Segment(0, 1)));
		detector.chains.get(0).steps.add(detector.new Segment(0, 1));
		detector.chains.get(0).steps.add(detector.new Segment(1, 2));
		detector.chains.get(0).steps.add(detector.new Segment(2, 3));
		detector.chains.get(0).steps.add(detector.new Segment(3, 4));
		for(int x = 0; x < 640; x++) {
			for(int y = 0; y < 480; y++) {
				Point p = new Point(x,y);
				double score = detector.chains.get(0).scoreByDistance(p);
				Imgproc.circle(image, p, 0, new Scalar(score,0,255-score));
			}
		}
		for(int i = 0; i < detector.lights.size(); i++) {
			Point center = detector.lights.get(i);
			Imgproc.circle(image, center , 7, new Scalar(0, 255, 0));
			Imgproc.putText(image, " "+i, center, 1, 1, new Scalar(0,255,0));
		}
		HighGui.imshow("test", image);
		HighGui.waitKey();
		System.exit(0);
	}
}
