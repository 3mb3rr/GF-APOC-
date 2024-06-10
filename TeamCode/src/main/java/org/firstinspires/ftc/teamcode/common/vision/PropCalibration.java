package org.firstinspires.ftc.teamcode.common.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;

@TeleOp
public class PropCalibration extends LinearOpMode {
    VisionPortal visionPortal;
    public PropColorCalibrationPipeline pipeline;

    @Override
    public void runOpMode() {
        pipeline = new PropColorCalibrationPipeline();
        startCamera();

        // Start streaming the camera to the dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Average HSV", pipeline.getAverageHSV());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Average HSV", pipeline.getAverageHSV());
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
    }
}
