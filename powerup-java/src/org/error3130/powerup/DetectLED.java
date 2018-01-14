package org.error3130.powerup;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;

public class DetectLED
{
   public static void main( String[] args )
   {
	   System.loadLibrary( Core.NATIVE_LIBRARY_NAME );

	   if(args.length > 0 ) {
		   Mat image = Imgcodecs.imread(args[0]);
		   HighGui.imshow("test", image);
		   HighGui.waitKey();
		   System.out.println("Key pressed");
	   }
	   HighGui.destroyWindow("test");
   }
}
