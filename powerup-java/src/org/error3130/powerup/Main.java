package org.error3130.powerup;

import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.error3130.powerup.DetectLED;

public class Main {
	public static void main( String[] args )
	{
		System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
		System.out.println("Library loaded: "+ Core.NATIVE_LIBRARY_NAME);

		if(args.length > 0 ) {
			Mat image = Imgcodecs.imread(args[0]);
			if(image.empty()) {
				System.err.println("Could not import file "+ args[0]);
				return;
			}

			HighGui.imshow("test", image);
			HighGui.waitKey();

			DetectLED detector = new DetectLED(
					80,
					Math.sqrt(image.size().area())/400,
					Math.sqrt(image.size().area())/16
				);

			List<Point> lights = detector.findLEDs(image);
			System.out.println("Lights detected: "+ lights.size());
			for(Point center: lights) {
				Imgproc.circle(image, center , 10, Scalar.all(255));
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
		return;
	}
}
