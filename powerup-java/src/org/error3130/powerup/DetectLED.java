package org.error3130.powerup;

import java.util.List;

import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class DetectLED
{
	double thresh;
	double minArea;
	double maxArea;

	public DetectLED(double brightnessThresh, double blobMinArea, double blobMaxArea) {
		this.maxArea = blobMaxArea;
		this.minArea = blobMinArea;
		this.thresh = brightnessThresh;
	}

	public List<Point> findLEDs(Mat image) {
		List<Point> lights = new java.util.ArrayList<Point>();

		Mat grey = new Mat();
		Mat bw = new Mat();
		
		Imgproc.cvtColor(image, grey, Imgproc.COLOR_BGR2GRAY);
		Imgproc.threshold(grey, bw, thresh , 255, 0);

		HighGui.imshow("test", bw);
		HighGui.waitKey();

		List<MatOfPoint> contours = new java.util.ArrayList<MatOfPoint>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(bw, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

		for (MatOfPoint cont : contours) {
			double area = Imgproc.contourArea(cont);
			System.out.println("Area: "+area);
			if(minArea < area && area  < maxArea) {
				Moments m = Imgproc.moments(cont);
				lights.add(new Point(m.m10/m.m00, m.m01/m.m00));
			}
		}
		return lights;
	}
}
