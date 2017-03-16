package org.usfirst.frc.team4454.robot;

import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.imgproc.*;

public class ShooterVisionPipeline implements VisionPipeline {

	// Process Parameters
	double[] hsvThresholdHue = {0.0, 255.0};
	double[] hsvThresholdSaturation = {0.0, 255.0};
	double[] hsvThresholdValue = {0.0, 255.0};

	double filterContoursMinArea = 40.0;
	double filterContoursMinPerimeter = 0;
	double filterContoursMinWidth = 5;
	double filterContoursMaxWidth = 1000;
	double filterContoursMinHeight = 5;
	double filterContoursMaxHeight = 1000;
	double[] filterContoursSolidity = {0, 100};
	double filterContoursMaxVertices = 1000000;
	double filterContoursMinVertices = 0;
	double filterContoursMinRatio = 3.0;
	double filterContoursMaxRatio = 10.0;

	// Outputs
	// This needs to be initialized or hsvThreshold fails with a null pointer exception/
	private Mat hsvThresholdOutput = new Mat(); 
	private Mat overlayOutput;
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

	public boolean foundTarget = false; // indicate whether you found the target
	int targetTop, targetBottom, targetLeft, targetRight, targetHeight, targetWidth;

	double targetDistance;


	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	public void process(Mat source0) {
		// Step HSV_Threshold0:
		Mat hsvThresholdInput = source0;

		overlayOutput = source0;

		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Note that the openCV routines
		// Step Find_Contours0:
		Mat findContoursInput = hsvThresholdOutput.clone();
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
		findContoursInput.release();
		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;

		filterContours(filterContoursContours, 
				filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, 
				filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, 
				filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, 
				filterContoursOutput);

		findTarget(filterContoursOutput);
	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThresholdOutput() {
		return hsvThresholdOutput;
	}

	public Mat overlayOutput() {
		return overlayOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	/**
	 * This method is a generated getter for the output of a Filter_Contours.
	 * @return ArrayList<MatOfPoint> output from Filter_Contours.
	 */
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}


	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param output The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
			Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
				new Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	 * @param input The image on which to perform the Distance Transform.
	 * @param type The Transform.
	 * @param maskSize the size of the mask.
	 * @param output The image in which to store the output.
	 */
	private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}


	/**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 * @param maxWidth maximum width
	 * @param minHeight minimum height
	 * @param maxHeight maximimum height
	 * @param Solidity the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
			double minPerimeter, double minWidth, double maxWidth, double minHeight, double
			maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
			minRatio, double maxRatio, List<MatOfPoint> output) {
		// final MatOfInt hull = new MatOfInt();
		output.clear();
		//operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;

			/***
			final double area = Imgproc.contourArea(contour);
			if (area < minArea) continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int)hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
			 ***/

			final double ratio = bb.width / (double)bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.add(contour);
		}
	}

	private void findTarget (List<MatOfPoint> inputContours) {
		int n = inputContours.size();
		int i, j;
		Rect r1, r2;
		double temp, aspectRatio;

		foundTarget = false;

		if (n >= 2) {
			for (i = 0; i < n; ++i) {
				r1 = Imgproc.boundingRect(inputContours.get(i));

				for (j = 0; j < n; ++j) {
					r2 = Imgproc.boundingRect(inputContours.get(j));

					// compute x center of r2 relative to r1
					temp = (r2.x + 0.5*r2.width) - r1.x;

					if ( (temp >= 0.0) && (temp <= r1.width) ) {

						targetTop    = Math.min(r1.y, r2.y);
						targetBottom = Math.max(r1.y+r1.height, r2.y+r2.height);

						targetLeft  = Math.min(r1.x, r2.x);
						targetRight = Math.max(r1.x+r1.width, r2.x+r2.width);

						targetHeight = targetBottom - targetTop;
						targetWidth  = targetRight - targetLeft;

						aspectRatio = (double)targetHeight / (double)targetWidth;

						if (Math.abs(aspectRatio - (10/15)) < 0.5) {
							foundTarget = true;

							Imgproc.rectangle(hsvThresholdOutput, 
									new Point(targetLeft, targetTop), 
									new Point(targetRight, targetBottom), 
									new Scalar(0, 0, 255));
							return;
						}
					}
				}
			}
		}
	}
}
