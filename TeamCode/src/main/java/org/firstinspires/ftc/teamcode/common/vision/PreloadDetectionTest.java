package org.firstinspires.ftc.teamcode.common.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.pathing.localization.Pose;

import org.firstinspires.ftc.teamcode.common.pathing.localization.AprilTagConstants;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import android.util.Size;

import java.util.List;

@TeleOp
public class PreloadDetectionTest extends LinearOpMode {
    private final robotHardware robot = robotHardware.getInstance();
//    VisionPortal visionPortal;
//    public AprilTagProcessor aprilTag;
//    public PropDetectionPipeline propDetectionPipeline;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        CenterstageConstants.IS_AUTO = false;
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);
        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            robot.read();
            robot.periodic();
            robot.write();
            telemetry.addData("joemama",robot.preloadDetectionPipeline.getPreloadedZone());
            telemetry.update();
        }
    }
}