
package org.firstinspires.ftc.teamcode.common.vision;
/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import android.util.Size;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class PropDetectionTest extends LinearOpMode
{
    VisionPortal visionPortal;
    public PropDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        telemetry= FtcDashboard.getInstance().getTelemetry();
        pipeline = new PropDetectionPipeline(0.3, 0.7, 0.65);
        startCamera();

        telemetry.setMsTransmissionInterval(50);
        //Hardware Mapping HERE etc.
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("0 == Error");
            telemetry.addLine("1 == Left");
            telemetry.addLine("2 == middle");
            telemetry.addLine("3 == Right");
            telemetry.addData("zone", pipeline.detectZone());
            telemetry.update();


        }
    }
    public void startCamera() {
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(800, 448))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(pipeline)
                .enableLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(pipeline, true);
    }

}
