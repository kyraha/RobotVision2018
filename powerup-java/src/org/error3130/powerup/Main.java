package org.error3130.powerup;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.error3130.powerup.DetectLED;
import org.error3130.powerup.DetectLED.Chain;

public class Main {
	static{
		System.loadLibrary( Core.NATIVE_LIBRARY_NAME );
		System.out.println("Library loaded: "+ Core.NATIVE_LIBRARY_NAME);
	}

	// Camera intrinsics, obtained by "camera calibration" (see [google] tutorial_interactive_calibration.html)
	// These numbers are for the MS Lifecam HD3000 #1 at 1280x720 mode
	static private double[] cameraDoubles = {1.1159304477424785e+03, 0., 6.2199798030884358e+02, 0.,
		    1.1159304477424785e+03, 3.8725727347891808e+02, 0., 0., 1.};
	static private double[] distortionDoubles = {1.2109074822929057e-01, -1.0293273066620283e+00, 0., 0.,
		    1.8601846077358326e+00};

	static MatOfPoint3f objectPoints;
	static Mat cameraMatrix = new Mat(3, 3, CvType.CV_32F);
	static MatOfDouble distCoeffs;


	public static void processFrame(Mat image) {
		double imageSize = Math.sqrt(image.size().area());

		MatOfPoint2f imagePoints = new MatOfPoint2f();

		Rect roiRect = new Rect(
				new Point(0.2*image.width(), 0.14*image.height()),
				new Size(0.6*image.width(), 0.24*image.height()));
		Imgproc.rectangle(image, roiRect.tl(), roiRect.br(), new Scalar(64,64,64));

		DetectLED detector = new DetectLED()
				.withThresh(128)
				.withMinArea(imageSize/400)
				.withMaxArea(imageSize/10)
				.withMaxSegment(image.width()/10);

		detector.findLEDs(image, roiRect)
				.findSegments()
				.findChains()
				.getCorners();

		MatOfPoint2f twoCorners = detector.getCorners();
		if(twoCorners == null) return;
		imagePoints.push_back(twoCorners);

		for(int i = 0; i < detector.lights.size(); i++) {
			Point center = detector.lights.get(i);
			Imgproc.circle(image, center , (int)(imageSize/70), new Scalar(255,0,255));
			Imgproc.putText(image, " "+i, center, 1, 1, new Scalar(0,0,255));
		}

		if(detector.chains.size() > 0) {
			Chain sp = detector.chains.get(0);
			if(sp.nodes.size() > 1) {
				int i = 0;
				Point2 pointA = new Point2(0,0);
				for(Point2 pointB: sp.nodes) {
					if(i > 0) {
						Imgproc.line(image, pointA, pointB, new Scalar(0, 255, 255));
					}
					pointA = pointB;
					i++;
				}
			}
		}

		roiRect = new Rect(
				new Point(0.05*image.width(), 0.6*image.height()),
				new Size(0.9*image.width(), 0.22*image.height()));
		Imgproc.rectangle(image, roiRect.tl(), roiRect.br(), new Scalar(64,64,64));

		detector.findLEDs(image, roiRect)
				.findSegments()
				.findChains();

		twoCorners = detector.getCorners();
		if(twoCorners == null) return;
		imagePoints.push_back(twoCorners);

		for(int i = 0; i < detector.lights.size(); i++) {
			Point center = detector.lights.get(i);
			Imgproc.circle(image, center , (int)(imageSize/70), new Scalar(255,0,255));
			Imgproc.putText(image, " "+i, center, 1, 1, new Scalar(0,0,255));
		}
		if(detector.chains.size() > 0) {
			Chain sp = detector.chains.get(0);
			if(sp.nodes.size() > 1) {
				int i = 0;
				Point2 pointA = new Point2(0,0);
				for(Point2 pointB: sp.nodes) {
					if(i > 0) {
						Imgproc.line(image, pointA, pointB, new Scalar(0, 255, 255));
					}
					pointA = pointB;
					i++;
				}
			}
		}
		List<Point> corners = imagePoints.toList();
		for(Point corner: corners) {
			Point a = new Point2(-imageSize/40, -imageSize/40).plus(corner);
			Point b = new Point2( imageSize/40,  imageSize/40).plus(corner);
			Imgproc.rectangle(image, a, b, new Scalar(0,255,0));
		}

		Mat rvec = new Mat();
		Mat tvec = new Mat();
		Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
		String text = String.format("Tvec %6.1f %6.1f %6.1f",
			tvec.get(0,0)[0],
			tvec.get(1,0)[0],
			tvec.get(2,0)[0]);
		Imgproc.putText(image, text, new Point(10,10), 1, 1, new Scalar(0,255,0));
		text = String.format("Rvec %8.3f %8.3f %8.3f",
				rvec.get(0,0)[0],
				rvec.get(1,0)[0],
				rvec.get(2,0)[0]);
		Imgproc.putText(image, text, new Point(10,40), 1, 1, new Scalar(0,255,0));
	}
	
	private static void loadCameraIntrinsics()
	{
		File file = new File("/home/mkyraha/tmp/calibrate/cameraParameters.xml");
		DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
		DocumentBuilder db;
		
		try {
			db = dbf.newDocumentBuilder();
			Document document = db.parse(file);
			NodeList dataNodes = document.getElementsByTagName("data");
			for(int nod=0; nod < dataNodes.getLength(); nod++) {
				Node data = dataNodes.item(nod);
				String[] numbers = data.getTextContent().trim().split("[ \n]+");
				switch(data.getParentNode().getNodeName()) {
				case "cameraMatrix":
					System.out.println("CamMat: ");
					for(int i=0; i<3; i++)
						for(int j=0; j<3; j++) {
							double num = Double.parseDouble(numbers[3*i+j]);
							cameraMatrix.put(i, j, num);
							System.out.println("  : "+num);
						}
					break;
				case "dist_coeffs":
					System.out.println("DistCo: " + data.getTextContent());
					List<Double> doubles = new ArrayList<Double>();
					for(String strNum: numbers) {
						doubles.add(Double.parseDouble(strNum));
						System.out.println("DistCo: " + strNum);
					}
					distCoeffs = new MatOfDouble();
					distCoeffs.fromList(doubles);
					break;
				}
			}
		} catch (ParserConfigurationException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (SAXException | IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
	}

	public static void main( String[] args )
	{
		loadCameraIntrinsics();

		objectPoints = new MatOfPoint3f();
		List<Point3> objectLeft = new ArrayList<Point3>();
		List<Point3> objectRight = new ArrayList<Point3>();
		List<Point3> objectPractice = new ArrayList<Point3>();
		// Practice field
		// 144 + 159
		objectPractice.add(new Point3(-65.00, -57.5, 303));
		objectPractice.add(new Point3(-35.50, -58.5, 303));
		objectPractice.add(new Point3(-50.00, -6.5, 146));
		objectPractice.add(new Point3(-21.00, -6.5, 146));
		// Left side
		objectLeft.add(new Point3(-89.44, -57, 300));
		objectLeft.add(new Point3(-55.09, -57, 300));
		objectLeft.add(new Point3(-71.02, -10, 146));
		objectLeft.add(new Point3(-36.91, -8, 146));
		// Right side
		objectRight.add(new Point3(55.09, -57, 300));
		objectRight.add(new Point3(89.44, -57, 300));
		objectRight.add(new Point3(36.91, -8, 146));
		objectRight.add(new Point3(71.02, -10, 146));
		objectPoints.fromList(objectPractice);

		distCoeffs = new MatOfDouble(distortionDoubles);
		cameraMatrix = new Mat(3, 3, CvType.CV_32F);
		for(int i=0; i<3; i++)
			for(int j=0; j<3; j++) {
				cameraMatrix.put(i, j, cameraDoubles[3*i+j]);
			}
		
		if(args.length > 0 ) {
			Mat image = Imgcodecs.imread(args[0]);
			if(image.empty()) {
				System.err.println("Could not import file "+ args[0]);
				System.err.println("Working dir: "+ System.getProperty("user.dir"));
				return;
			}
			HighGui.imshow("test", image);
			HighGui.waitKey();

			processFrame(image);

			HighGui.imshow("test", image);
			HighGui.waitKey(0);
		}
		else {
			System.out.println("File name is missing. Trying to feed from camera");

			Mat frame = new Mat();
			VideoCapture cap = new VideoCapture();
			cap.open(0);

			if(cap.isOpened()) {
				// HD-3000 min exposure is 5, Tested: 5, 10, 20, 39, 78, 156, 312, 625...
				cap.set(Videoio.CAP_PROP_FRAME_WIDTH, 1280);
				cap.set(Videoio.CAP_PROP_FRAME_HEIGHT, 720);
				cap.set(Videoio.CAP_PROP_FPS, 4);
				cap.set(Videoio.CAP_PROP_AUTO_EXPOSURE, 1);
				cap.set(Videoio.CAP_PROP_EXPOSURE, 156);
				System.out.println("Exposure: "+ cap.get(Videoio.CAP_PROP_EXPOSURE));
				do {
					cap.read(frame);
					processFrame(frame);
					HighGui.imshow("test", frame);
				} while (HighGui.waitKey(10) < 0);
			}
			else {
				System.err.println("Cannot open a video stream");
			}
		}

		HighGui.destroyAllWindows();
		System.out.println("All done");
		System.exit(0);
	}
}
