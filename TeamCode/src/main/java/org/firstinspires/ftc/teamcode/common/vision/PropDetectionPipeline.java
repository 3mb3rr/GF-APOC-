package org.firstinspires.ftc.teamcode.common.vision;


import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
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
    Rect leftRegion;
    Rect rightRegion;
    Rect centerRegion;

    public PropDetectionPipeline(double leftBorder, double rightBorder, double topBorder) {
        frameList = new ArrayList<>();
        leftLimit=(int)(leftBorder*800);
        rightLimit=(int)((rightBorder)*800);
        topLimit = (int)((1-topBorder)*448);
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    @Override
    public Mat processFrame(Mat input,  long captureTimeNanos) {
        leftRegion = new Rect(new Point(0, topLimit),new Point(leftLimit, 448));
        centerRegion = new Rect(new Point(leftLimit, topLimit),new Point((rightLimit),448));
        rightRegion = new Rect(new Point(rightLimit,topLimit),new Point((800),448));
        input.copyTo(output);

        // convert it from an RGB to HSV
        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);

        // HSV filtering limits
        // yo these new values are CRAZY GOOD
        Scalar lowHSVRed = new Scalar(0, 130, 80);
        Scalar highHSVRed = new Scalar(10 ,255, 255);

        Scalar lowHSVBlue = new Scalar(105, 100, 100);
        Scalar highHSVBlue = new Scalar(115 ,255, 255);

        // Avneet's Tuned Values
//        Scalar lowHSVRed = new Scalar(170, 100, 100);
//        Scalar highHSVRed = new Scalar(180 ,255, 255);
//
//        Scalar lowHSVBlue = new Scalar(100, 100, 100);
//        Scalar highHSVBlue = new Scalar(110 ,255, 255);

        // filter for red
        if(CenterstageConstants.ALLIANCE == Location.RED) Core.inRange(inputHSV, lowHSVRed, highHSVRed, redDetection);
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
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.GREEN);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 15);
        canvas.drawRect(makeGraphicsRect(leftRegion, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(rightRegion, scaleBmpPxToCanvasPx), rectPaint);
        canvas.drawRect(makeGraphicsRect(centerRegion, scaleBmpPxToCanvasPx), rectPaint);
    }
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
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