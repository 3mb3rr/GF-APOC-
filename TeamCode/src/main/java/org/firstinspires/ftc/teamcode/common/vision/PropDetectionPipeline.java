package org.firstinspires.ftc.teamcode.common.vision;


import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class PropDetectionPipeline implements VisionProcessor {
    ArrayList<double[]> frameList;
    Mat output = new Mat();
    Scalar middle = new Scalar(0, 0, 0);
    Mat redDetection = new Mat();
    Mat inputHSV = new Mat();
    Mat deNoised = new Mat();
    Mat leftMat, centerMat, rightMat = new Mat();
    int leftLimit = 0;
    int rightLimit = 0;
    int topLimit = 0;
    int leftPixels, centerPixels, rightPixels = 0;
    int zone = 0;
    public enum colour {
        blue,
        red
    }
    colour Colour = colour.red;
    public PropDetectionPipeline(double leftBorder, double rightBorder, double topBorder, colour colour) {
        frameList = new ArrayList<>();
        leftLimit=(int)(leftBorder*800);
        rightLimit=(int)((rightBorder)*800);
        topLimit = (int)((1-topBorder)*448);
        Colour = colour;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public Mat processFrame(Mat input,  long captureTimeNanos) {
        Rect leftRegion = new Rect(new Point(0, topLimit),new Point(leftLimit, 448));
        Rect centerRegion = new Rect(new Point(leftLimit, topLimit),new Point((rightLimit),448));
        Rect rightRegion = new Rect(new Point(rightLimit,topLimit),new Point((800),448));
        input.copyTo(output);

        // convert it from an RGB to HSV
        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);

        // HSV filtering limits
        // Avneet's Tuned Values
        Scalar lowHSVRed = new Scalar(170, 100, 100);
        Scalar highHSVRed = new Scalar(180 ,255, 255);

        Scalar lowHSVBlue = new Scalar(100, 100, 100);
        Scalar highHSVBlue = new Scalar(110 ,255, 255);

        // filter for red
        if(Colour == colour.red) Core.inRange(inputHSV, lowHSVRed, highHSVRed, redDetection);
        else Core.inRange(inputHSV, lowHSVBlue, highHSVBlue, redDetection);
        noiseReduction(redDetection, deNoised);
        leftMat = deNoised.submat(leftRegion);
        centerMat = deNoised.submat(centerRegion);
        rightMat = deNoised.submat(rightRegion);
        leftPixels = countWhitePixels(leftMat);
        rightPixels = countWhitePixels(rightMat);
        centerPixels = countWhitePixels(centerMat);


        redDetection.release();
        centerMat.release();
        rightMat.release();
        leftMat.release();
        deNoised.release();
        inputHSV.release();

        Imgproc.line(output, new Point(leftLimit, 0), new Point(leftLimit, 448), new Scalar(0, 0, 255), 5);
        Imgproc.line(output, new Point(rightLimit, 0), new Point(rightLimit, 448), new Scalar(0, 0, 255), 5);
        Imgproc.line(output, new Point(0, topLimit), new Point(800, topLimit), new Scalar(0, 0, 255), 5);

        return output;

    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

    private void noiseReduction(Mat input, Mat output)
    {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.dilate(output, output, dilateElement);
    }
    public int detectZone(){
        if((centerPixels>leftPixels) && (centerPixels>rightPixels)){
            return 2;
        }
        if((rightPixels>centerPixels) && (rightPixels>leftPixels)){
            return 3;
        }
        return 1;
    }
    public int countWhitePixels(Mat mat) {
        // Assuming the input mat is binary (white pixels are 255, and black pixels are 0)
        Mat binaryMat = new Mat();
        Imgproc.threshold(mat, binaryMat, 254, 255, Imgproc.THRESH_BINARY);

        int whitePixels = Core.countNonZero(binaryMat);
        binaryMat.release();

        return whitePixels;
    }
}