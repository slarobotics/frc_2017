package org.usfirst.frc.team4454.robot;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

import org.opencv.core.*;
// import org.opencv.core.Core.*;
// import org.opencv.features2d.FeatureDetector;
// import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
// import org.opencv.objdetect.*;


public class LineVisionPipeline implements VisionPipeline {

	private Mat lineOutput; // = new Mat();
	
	// default blue
	int r = 0;
	int g = 0;
	int b = 255;
	
	int width = 1;

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	public void process(Mat source0) {
		// Step HSV_Threshold0:
		lineOutput = source0;
		Imgproc.line(lineOutput, new Point(160, 0), new Point(160, 240), new Scalar(b, g, r), width);
	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	
	public Mat lineOutput() {
		return lineOutput;
	}
}
