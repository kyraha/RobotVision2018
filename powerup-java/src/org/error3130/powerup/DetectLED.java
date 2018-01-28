package org.error3130.powerup;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
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
		public Segment(int a, int b) { A=a; B=b; }
		public String toString() {
			return "[" + A + ", " + B + "]";
		}

		public Point pointA() { return lights.get(A); }
		public Point pointB() { return lights.get(B); }
		public Point vector() {
			return new Point(pointB().x-pointA().x, pointB().y - pointA().y);
		}
		public double length() {
			return Math.sqrt(vector().dot(vector()));
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
		public double crossMag(Segment other) {
			double x1 = this.vector().x;
			double y1 = this.vector().y;
			double x2 = other.vector().x;
			double y2 = other.vector().y;
			return x1*y2 - y1*x2; // Magnitude of the cross product
		}
	}

	class Chain {
		public List<Segment> steps = new ArrayList<Segment>();
		public double totalScore = 0;

		public Chain(Segment s1) {steps.add(s1);}

		public boolean bestMatch(double limit) {
			double bestScore = Double.MAX_VALUE;
			int bestLight = -1;
			int myEnd = steps.get(steps.size()-1).B;
			List<Integer> myPoints = new ArrayList<>();
			for(Segment s: steps) { myPoints.add(s.A); }
			myPoints.add(myEnd);
			for(int p = 0; p < lights.size(); p++) {
				if(myPoints.contains(p)) continue;
				Segment s = new Segment(myEnd,p);
				double score = 0;
				for(Segment step: steps) {
					double mlen = step.length();
					double slen = s.length();
					score += Math.abs(mlen-slen)/mlen;
					// A property of the cross product: |a X b| = |a||b||sin|
					score += Math.abs(step.crossMag(s)/(mlen*slen));
					// A property of the dot product: a * b = |a||b||cos|
					score += Math.abs(step.vector().dot(s.vector())/(mlen*slen) - 1.0);
				}
				if(score < bestScore) {
					bestScore = score;
					bestLight = p;
				}
			}
			if(bestLight >= 0 && (steps.size()==1 || bestScore < limit*totalScore/steps.size())) {
				steps.add(new Segment(myEnd, bestLight));
				totalScore += bestScore;
				return true;
			}
			return false;
		}
	}

	private double thresh;
	private double minArea;
	private double maxArea;
	private double maxSeg;
	public List<Point> lights = new ArrayList<Point>();
	public List<Chain> chains = new ArrayList<Chain>();

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
		// Every single line segment is a candidate to build a chain upon it
		// So let's seed the starting points for all potential chains
		for (int i=0; i < lights.size(); i++) {
			for (int j=0; j < lights.size(); j++) {
				if(i != j && new Segment(i, j).length() < maxSeg) {
					chains.add(new Chain(new Segment(i, j)));
				}
			}
		}
		return this;
	}

	public DetectLED findChains() {
		double bestSoFar = Double.MAX_VALUE;
		System.out.println("Starting with: "+bestSoFar);
		for (Chain c: chains) {
			boolean found;
			double avgScore = 0;
			// Build the chain with up to 8 steps (9 total, or 10 points)
			for (int i = 0; i < 8; i++) {
				found = c.bestMatch(15.0);
				avgScore = c.totalScore/c.steps.size();
				if(!found) break;
			}
			c.totalScore += Math.abs(9 - c.steps.size())/9;
			avgScore = c.totalScore/c.steps.size();
			if(avgScore < bestSoFar) {
				bestSoFar = avgScore;
				System.out.println("chain size: " + c.steps.size() + ", score: " + avgScore);
			}
		}
		Collections.sort(chains, new Comparator<Chain>() {
			public int compare(Chain a, Chain b) {
				if (a.totalScore < b.totalScore) return -1;
				if (a.totalScore > b.totalScore) return 1;
				return 0;
			}
		});
		
		return this;
	}

}
