package org.error3130.powerup;

import org.opencv.core.Point;

public class Point2 extends Point {
	public Point2(double d, double e) {super(d, e);}
	public Point2(Point other) {super(other.x, other.y);}

	public double norm() {
		return Math.sqrt(x*x+y*y);
	}

	public Point2 plus(Point other) {
		return new Point2(x+other.x, y+other.y);
	}
	public Point2 negative() {
		return new Point2(-x, -y);
	}
	public Point2 minus(Point2 other) {
		return this.plus(other.negative());
	}

	public Point2 scale(double m) {
		return new Point2(x*m, y*m);
	}

	public double distance(Point other) {
		double dx = x - other.x;
		double dy = y - other.y;
		return Math.sqrt(dx*dx + dy*dy);
	}

	public double cross(Point other){
	   return x*other.y - y*other.x;
	}

	public double distanceToLine( Point2 linePoint, Point2 lineNormale ){
	   //end is lineNormale;
	   Point2 relative = this.minus(linePoint);

	   //¿do you see the triangle?
	   double area = relative.cross(lineNormale);
	   return area / lineNormale.norm();
	}
}
