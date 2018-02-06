package org.error3130.powerup;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class DetectLED
{
	public class Chain {
		public List<Point2> nodes = new ArrayList<Point2>();
		public List<Integer> nodeIndex = new ArrayList<Integer>(maxNodes);

		public void addNode(int lightNum) {
			if(nodes.size() < maxNodes) {
				nodeIndex.add(lightNum);
				nodes.add(lights.get(lightNum));
			}
		}
		
		public Point2 base() {
			if(nodes.size() > 1) {
				return nodes.get(1).minus(nodes.get(0));
			}
			else {
				return new Point2(0,0);
			}
		}

		public Chain(int i, int j) {
			addNode(i);
			addNode(j);
		}

		double score() {
			//MatOfPoint ppp = new MatOfPoint(a);
			Mat points = new Mat(nodes.size(),2,CvType.CV_32FC1);
			Mat line = new Mat(4,1,CvType.CV_32FC1);

			for(int i = 0; i < nodes.size(); i++) {
				points.put(i, 0, nodes.get(i).x);
				points.put(i, 1, nodes.get(i).y);
			}

			Imgproc.fitLine(points, line, Imgproc.CV_DIST_L2, 0, 0.01, 0.01);
			Point2 lineNormale = new Point2(line.get(0, 0)[0], line.get(1, 0)[0]);
			Point2 linePoint = new Point2(line.get(2, 0)[0], line.get(3, 0)[0]);

			double score = 0;
			for(int i = 0; i < nodes.size(); i++) {
				score += Math.abs(nodes.get(i).distanceToLine(linePoint, lineNormale));
				if(i > 0)
					score += Math.abs(nodes.get(i).minus(nodes.get(i-1)).dot(base()) - 1);
			}
			return score / (base().norm() * nodes.size()) + maxNodes*(maxNodes - nodes.size());
		}

		double scoreByDistance(Point2 p) {
			double score = 0;
			Point2 end = nodes.get(nodes.size()-1); // Last point so far
			Point2 tail = p.minus(end); // Vector from the end to the point p

			// If the new candidate is too far then bail out
			if(tail.norm() > maxSeg) return Double.MAX_VALUE;

			// Expect the new tail be like base or half or twice of it
			score += base().norm() / base().minus(tail).norm();
			score += base().norm() / base().scale(0.5).minus(tail).norm();
			score += base().norm() / base().scale(2.0).minus(tail).norm();
			return 255.0 / score;
		}

		public boolean bestMatch(double limit) {
			double bestScore = Double.MAX_VALUE;
			int bestLight = -1;
			for(int p = 0; p < lights.size(); p++) {
				if(nodeIndex.contains(p)) continue;
				double score = scoreByDistance(lights.get(p));
				if(score < bestScore) {
					bestScore = score;
					bestLight = p;
				}
			}
			if(bestLight >= 0) {
				addNode(bestLight);
				return true;
			}
			return false;
		}
	}

	private double thresh = 128;
	private double minArea = 1;
	private double maxArea = 64;
	private double maxSeg = 100;
	private int maxNodes = 10;
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

		// Convert ROI of the original frame to a gray image
		Imgproc.cvtColor(new Mat(image, roiRect), grey, Imgproc.COLOR_BGR2GRAY);
		// Then highlight the bright pixels
		Imgproc.threshold(grey, bw, thresh , 255, 0);

		// Find blobs of bright pixels keeping in mind the ROI offset
		// Pay attention to EXTERNAL contours only, ignore nested contours.
		List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		Imgproc.findContours(bw, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE, roiRect.tl());

		for (MatOfPoint cont: contours) {
			double area = Imgproc.contourArea(cont);
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
				if(i != j && lights.get(i).minus(lights.get(j)).norm() < maxSeg) {
					chains.add(new Chain(i, j));
				}
			}
		}
		return this;
	}

	public DetectLED findChains() {
		for (Chain c: chains) {
			boolean found;
			// Build each chain with up to 8 steps (9 steps total, or 10 points)
			for (int i = 0; i < 8; i++) {
				found = c.bestMatch(300.0);
				if(!found) break;
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
