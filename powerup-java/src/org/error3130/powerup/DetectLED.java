package org.error3130.powerup;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class DetectLED
{
	public static double segLength(Point o, Point p) {
		double dx = o.x - p.x;
		double dy = o.y - p.y;
		return Math.sqrt(dx*dx + dy*dy);
	}

	public class Segment {
		int A, B;
		public Segment(int a, int b) { A=a; B=b; }
		public String toString() {
			return "[" + A + "," + B + "]";
		}

		public Point2 pointA() { return lights.get(A); }
		public Point2 pointB() { return lights.get(B); }
		public Point2 vector() {
			return pointB().minus(pointA());
		}
		public double length() {
			return Math.sqrt(vector().dot(vector()));
		}
		public double crossMag(Point otherVec) {
			return this.vector().cross(otherVec);
		}
		public double crossMag(Segment other) {
			Point vec = other.vector();
			return crossMag(vec);
		}
	}

	public class Chain {
		public List<Segment> steps = new ArrayList<Segment>();
		public double totalScore = 0;

		public Chain(Segment s1) {steps.add(s1);}

		double score() {
			Mat points = new Mat(steps.size()+1,2,CvType.CV_32FC1);
			Mat line = new Mat(4,1,CvType.CV_32FC1);
			Point2 base = steps.get(0).vector();

			points.put(0, 0, lights.get(steps.get(0).A).x);
			points.put(0, 1, lights.get(steps.get(0).A).y);
			for(int i = 0; i < steps.size(); i++) {
				points.put(i+1, 0, lights.get(steps.get(i).B).x);
				points.put(i+1, 1, lights.get(steps.get(i).B).y);
			}

			Imgproc.fitLine(points, line, Imgproc.CV_DIST_L2, 0, 0.01, 0.01);
			Point2 lineNormale = new Point2(line.get(0, 0)[0], line.get(1, 0)[0]);
			Point2 linePoint = new Point2(line.get(2, 0)[0], line.get(3, 0)[0]);

			double score = Math.abs(lights.get(steps.get(0).A).distanceToLine(linePoint, lineNormale));
			for(int i = 0; i < steps.size(); i++) {
				score += Math.abs(lights.get(steps.get(i).B).distanceToLine(linePoint, lineNormale));
				score += Math.abs(steps.get(i).vector().dot(base) - 1);
			}
			return score / (base.norm() * steps.size()) + 10*(9 - steps.size());
		}

		double scoreGravitational(Point p) {
			Point o = lights.get(steps.get(steps.size()-1).B);
			Point gravector = new Point(0,0);
			for(Segment step: steps) {
				Point svec = step.vector();
				for(double M = 0.5; M <= 2; M += 0.5) {
					Point gpoint = new Point(o.x + M*svec.x, o.y + M*svec.y);
					Point gvec = new Point(gpoint.x - p.x, gpoint.y - p.y);
					double r = segLength(gpoint, p);
					if(r > 0) {
						gravector.x += gvec.x / Math.abs(M*r*r*r);
						gravector.y += gvec.y / Math.abs(M*r*r*r);
					}
				}
			}
			return 0.01 / (gravector.x * gravector.x + gravector.y * gravector.y);
		}

		double scoreByDistance(Point2 p) {
			double score = 0;
			Point2 o = lights.get(steps.get(steps.size()-1).B);
			Point2 v = p.minus(o); // This is a vector from the end to the point p
			Point2 s = steps.get(0).vector();
			score += steps.get(0).length() / s.minus(v).norm();
			s.x /= 2.0; s.y /= 2.0;
			score += steps.get(0).length() / s.minus(v).norm();
			s.x *= 4.0; s.y *= 4.0;
			score += steps.get(0).length() / s.minus(v).norm();
			return 255.0 / score;
		}

		double score(Point p) {
			Point o = lights.get(steps.get(steps.size()-1).B);
			Point v = new Point(p.x-o.x, p.y-o.y); // This is a vector from the end to the point p
			double baseLen = steps.get(0).length();
			double newLen = segLength(o,p);
			double score = 0;
			// A property of the cross product: |a X b| = |a||b||sin|
			score += Math.abs(steps.get(0).crossMag(v)/(baseLen*newLen)) * 512;
			// A property of the dot product: a * b = |a||b||cos|
			score += Math.abs(steps.get(0).vector().dot(v)/(baseLen*newLen) - 1.0) * 255;
			for(Segment step: steps) {
				double mlen = step.length();
				double slen = segLength(o,p);
				// Check if the distance is close to N times the step
				double offset = (slen % (0.5*mlen)) / (0.5*mlen);
				score += Math.abs(offset < 0.5 ? offset : 1.0 - offset);
				// Add score if the point is too far
				score += Math.abs(slen - mlen) / mlen;
			}
			return score;
		}

		public boolean bestMatch(double limit) {
			double bestScore = Double.MAX_VALUE;
			int bestLight = -1;
			int myEnd = steps.get(steps.size()-1).B;
			List<Integer> myPoints = new ArrayList<>();
			for(Segment s: steps) { myPoints.add(s.A); }
			myPoints.add(myEnd);
			for(int p = 0; p < lights.size(); p++) {
				if(myPoints.contains(p)) continue;
				double score = scoreByDistance(lights.get(p));
				if(score < bestScore) {
					bestScore = score;
					bestLight = p;
				}
			}
			if(bestLight >= 0 && (steps.size()==1 || bestScore < limit)) {
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
	public List<Point2> lights = new ArrayList<Point2>();
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

	public DetectLED findLEDs(Mat image, Rect roiRect) {
		// Reset all previously found LEDs if any
		lights = new ArrayList<Point2>();

		Mat grey = new Mat();
		Mat bw = new Mat();

		Mat roi = new Mat(image, roiRect);
		
		Imgproc.cvtColor(roi, grey, Imgproc.COLOR_BGR2GRAY);
		Imgproc.threshold(grey, bw, thresh , 255, 0);

		//HighGui.imshow("test", bw);
		//HighGui.waitKey();

		// Pay attention to EXTERNAL contours only, ignore nested contours.
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(bw, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE, roiRect.tl());

		for (MatOfPoint cont: contours) {
			double area = Imgproc.contourArea(cont);
			//System.out.println("Area: "+area);
			if(minArea < area && area  < maxArea) {
				Moments m = Imgproc.moments(cont);
				lights.add(new Point2(m.m10/m.m00, m.m01/m.m00));
			}
		}
		return this;
	}
	
	public DetectLED findSegments() {
		// Reset all previously found chains of segments if any
		chains = new ArrayList<Chain>();
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
				found = c.bestMatch(300.0);
				avgScore = c.totalScore/c.steps.size();
				if(!found) break;
			}
			c.totalScore += Math.abs(9 - c.steps.size()) * 60;
			avgScore = c.totalScore/c.steps.size();
			if(avgScore < bestSoFar) {
				bestSoFar = avgScore;
				System.out.println("chain size: " + c.steps.size() + ", score: " + c.score());
			}
		}
		Collections.sort(chains, new Comparator<Chain>() {
			public int compare(Chain a, Chain b) {
				if (a.score() < b.score()) return -1;
				if (a.score() > b.score()) return 1;
				return 0;
			}
		});
		
		return this;
	}

}
