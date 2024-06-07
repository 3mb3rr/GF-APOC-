package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class PropColorCalibrationPipeline implements VisionProcessor {
    private Mat output = new Mat();
    private Mat inputHSV = new Mat();
    private Mat centerMat = new Mat();
    private Scalar averageHSV = new Scalar(0, 0, 0);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        Rect centerRegion = new Rect(new Point(420, 260), new Point(380, 240));
        input.copyTo(output);

        // Convert the input frame from RGB to HSV
        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);

        centerMat = inputHSV.submat(centerRegion);

        // Calculate the average HSV values for the center region
        averageHSV = calculateAverageHSV(centerMat);

        // Draw a rectangle around the center region
        Imgproc.rectangle(output, new Point(420, 260), new Point(380, 240), new Scalar(255, 0, 0), 5);
        // Release temporary mats
        centerMat.release();
        inputHSV.release();

        return output;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}

    private Scalar calculateAverageHSV(Mat input) {
        // Split the HSV image into individual channels
        ArrayList<Mat> hsvChannels = new ArrayList<>(3);
        Core.split(input, hsvChannels);

        Mat hChannel = hsvChannels.get(0);
        Mat sChannel = hsvChannels.get(1);
        Mat vChannel = hsvChannels.get(2);

        // Calculate the circular mean for the H channel
        double hueMean = calculateCircularMean(hChannel, 180);

        // Calculate the arithmetic mean for the S and V channels
        double saturationMean = Core.mean(sChannel).val[0];
        double valueMean = Core.mean(vChannel).val[0];

        // Release the Mats
        hChannel.release();
        sChannel.release();
        vChannel.release();

        // Return the average HSV as a Scalar
        return new Scalar(hueMean, saturationMean, valueMean);
    }

    private double calculateCircularMean(Mat hChannel, int max) {
        double W = (2 * Math.PI) / max;
        double sumCos = 0;
        double sumSin = 0;
        int totalPixels = hChannel.rows() * hChannel.cols();

        // Check if hChannel is not empty to avoid IndexOutOfBoundsException
        if (totalPixels == 0) {
            return 0;
        }

        for (int i = 0; i < hChannel.rows(); i++) {
            for (int j = 0; j < hChannel.cols(); j++) {
                double[] hueArr = hChannel.get(i, j);
                if (hueArr != null && hueArr.length > 0) {
                    double hue = hueArr[0];
                    sumCos += Math.cos(hue * W);
                    sumSin += Math.sin(hue * W);
                }
            }
        }

        double xMean = sumCos / totalPixels;
        double yMean = sumSin / totalPixels;

        double circularMean = Math.atan2(yMean, xMean) / W;
        if (circularMean < 0) {
            circularMean += max;
        }

        return circularMean;
    }

    public Scalar getAverageHSV() {
        return averageHSV;
    }
}
