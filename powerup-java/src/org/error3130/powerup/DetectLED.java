package org.error3130.powerup;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class DetectLED
{
	class Line {
		double slope;
		double intercept;
		public Line(double m, double b) {slope=m; intercept=b;}
		public String toString() {
			return "(" + slope + ", " + intercept + ")";
		}
	}

	class Segment {
		int A, B;
		public Segment(int a, int b) {A=a; B=b;}
		public double length() {
			double dx = lights.get(B).x - lights.get(A).x;
			double dy = lights.get(B).y - lights.get(A).y;
			return Math.sqrt(dx*dx + dy*dy); 
		}
		public Line line() {
			if(lights.get(A).x != lights.get(B).x) {
				double slope = (lights.get(B).y-lights.get(A).y)/(lights.get(B).x-lights.get(A).x);
				double intercept = lights.get(A).y + slope*lights.get(A).x;
				return new Line( slope, intercept);
			}
			else {
				// In case if the is vertical the Y-intercept doesn't make sense
				// So if we store the X-intercept it will be enough to describe the line
				return new Line( Double.POSITIVE_INFINITY, lights.get(A).x );
			}
		}
	}

	class SegmentPair {
		public Segment a, b;
		public SegmentPair(Segment s1, Segment s2) {a=s1; b=s2;}
	}

	private double thresh;
	private double minArea;
	private double maxArea;
	private double maxSeg;
	public List<Point> lights = new ArrayList<Point>();
	public List<Segment> segments = new ArrayList<Segment>();
	public List<SegmentPair> segmentPairs = new ArrayList<SegmentPair>();

	public DetectLED(double brightnessThresh, double blobMinArea, double blobMaxArea) {
		this.maxArea = blobMaxArea;
		this.minArea = blobMinArea;
		this.thresh = brightnessThresh;
	}

	public DetectLED() {}

	public DetectLED withThresh(double x) { this.thresh = x; return this; }
	public DetectLED withMinArea(double x) { this.minArea = x; return this; }
	public DetectLED withMaxArea(double x) { this.maxArea = x; return this;	}
	public DetectLED withMaxSegment(double x) { this.maxSeg = x; return this; }

	public DetectLED findLEDs(Mat image) {

		Mat grey = new Mat();
		Mat bw = new Mat();
		
		Imgproc.cvtColor(image, grey, Imgproc.COLOR_BGR2GRAY);
		Imgproc.threshold(grey, bw, thresh , 255, 0);

		//HighGui.imshow("test", bw);
		//HighGui.waitKey();

		// Pay attention to EXTERNAL contours only, ignore nested contours.
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(bw, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		for (MatOfPoint cont: contours) {
			double area = Imgproc.contourArea(cont);
			//System.out.println("Area: "+area);
			if(minArea < area && area  < maxArea) {
				Moments m = Imgproc.moments(cont);
				lights.add(new Point(m.m10/m.m00, m.m01/m.m00));
			}
		}
		return this;
	}
	
	public DetectLED findSegments() {
		for (int i=0; i < lights.size(); i++) {
			for (int j=i+1; j < lights.size(); j++) {
				Segment seg = new Segment(i, j);
				if(seg.length() < maxSeg) segments.add(seg);
			}
		}
		return this;
	}

	public DetectLED findLines() {
		for (int i=0; i < segments.size(); i++) {
			Line li = segments.get(i).line();
			for (int j=i+1; j < segments.size(); j++) {
				Line lj = segments.get(j).line();
				if(
						Math.abs(li.slope-lj.slope) < 0.1
					&&  Math.abs(li.intercept-lj.intercept) < 1.0
						) {
					segmentPairs.add(new SegmentPair(segments.get(i), segments.get(j)));
					System.out.println("Line: "+li+" and "+lj);
					
				}
			}
		}
		return this;
	}
}
