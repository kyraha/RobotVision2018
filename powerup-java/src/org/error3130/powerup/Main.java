package org.error3130.powerup;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.error3130.powerup.DetectLED;
import org.error3130.powerup.DetectLED.Segment;
import org.error3130.powerup.DetectLED.SegmentPair;

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

			DetectLED detector = new DetectLED()
					.withThresh(160)
					.withMinArea(imageSize/400)
					.withMaxArea(imageSize/16)
					.withMaxSegment(imageSize/10)
					.findLEDs(image)
					.findSegments()
					.findLines();

			System.out.println("Lights detected: "+ detector.lights.size());
			for(Point center: detector.lights) {
				Imgproc.circle(image, center , (int)(imageSize/70), new Scalar(255,0,255));
			}
			HighGui.imshow("test", image);
			HighGui.waitKey();

			System.out.println("Segments detected: "+ detector.segments.size());
			System.out.println("Segment pairs detected: "+ detector.segmentPairs.size());
//			for(Segment seg: detector.segments) {
//				Imgproc.line(image, detector.lights.get(seg.A), detector.lights.get(seg.B), new Scalar(0, 255, 255));
//			}

			int color = 0;
			for(SegmentPair sp: detector.segmentPairs) {
				Imgproc.line(image,
						detector.lights.get(sp.a.A),
						detector.lights.get(sp.a.B),
						new Scalar(255, color, 200));
				Imgproc.line(image,
						detector.lights.get(sp.b.A),
						detector.lights.get(sp.b.B),
						new Scalar(255, color, 200));
				color+=29;
				if(color > 255) color -= 256;
			}

			HighGui.imshow("test", image);
			HighGui.waitKey();

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
