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

			DetectLED detector = new DetectLED()
					.withThresh(160)
					.withMinArea(imageSize/400)
					.withMaxArea(imageSize/16)
					.withMaxSegment(imageSize/10)
					.findLEDs(image)
					.findSegments()
					.findChains();

			System.out.println("Lights detected: "+ detector.lights.size());
			for(int i = 0; i < detector.lights.size(); i++) {
				Point center = detector.lights.get(i);
				Imgproc.circle(image, center , (int)(imageSize/70), new Scalar(255,0,255));
				Imgproc.putText(image, " "+i, center, 1, 1, new Scalar(0,0,255));
			}
			HighGui.imshow("test", image);
			HighGui.waitKey();

			System.out.println("Chains detected: "+ detector.chains.size());

			int color = 0;
			for(Chain sp: detector.chains) {
				if(sp.steps.size() > 6) {
					for(Segment seg: sp.steps) {
						Imgproc.line(image, seg.pointA(), seg.pointB(), new Scalar(color, 255, 255));
					}
					System.out.println(sp.steps +" "+ sp.totalScore);
					HighGui.imshow("test", image);
					int key = HighGui.waitKey(0);
					color+=29;
					if(color > 200) color -= 200;
					if(key == 27) break;
				}
			}

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
