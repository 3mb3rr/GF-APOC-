package org.firstinspires.ftc.teamcode.opmode.testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToSpikePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToSpikePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.slideToRow;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.followPath;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.outtakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.stopIntake;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.v4BarToHeight;
import org.firstinspires.ftc.teamcode.common.pathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.*;
import java.util.function.BooleanSupplier;


@Autonomous
@Disabled
public class AutoRedFarLinear extends LinearOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private double currentTime = 0;
    private double lastTime = 0.0;
    private double loopTime = 0;
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toStackMiddle;
    private Path toStackLeftFromBB;
    private Path toStrafeAtLeftStack;
    private Path toBackboard;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toPark;


    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() > 26;

    VisionPortal visionPortal;
    public PropDetectionPipeline pipeline;
    private int zone;
    private Location randomization;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        pipeline = new PropDetectionPipeline(0.3, 0.7, 0.65);
        startCamera();


        telemetry.setMsTransmissionInterval(50);


        toSpikeMiddle = new Path(new BezierLine(new Point(-39, -58, Point.CARTESIAN), new Point(-55, -21.5, Point.CARTESIAN)));
        toSpikeMiddle.setConstantHeadingInterpolation(Math.toRadians(0));
        toSpikeLeft = new Path(new BezierLine(new Point(-39, -58, Point.CARTESIAN), new Point(-60, -17, Point.CARTESIAN)));
        toSpikeLeft.setConstantHeadingInterpolation(Math.toRadians(-44));
        toSpikeRight = new Path(new BezierLine(new Point(-39, -58, Point.CARTESIAN), new Point(-39.4, -34.6, Point.CARTESIAN)));
        toSpikeRight.setConstantHeadingInterpolation(Math.toRadians(0));

        toBackboard = new Path(new BezierCurve(new Point(-57, -19,Point.CARTESIAN), new Point(-27,-5,Point.CARTESIAN), new Point(30,0,Point.CARTESIAN),new Point(43,-35,Point.CARTESIAN)));
        toBackboard.setConstantHeadingInterpolation(Math.toRadians(0));

        toBackboardMiddle = new Path(new BezierLine(new Point(43, -35, Point.CARTESIAN), new Point(47.5, -36, Point.CARTESIAN))); //new Point(21, 0, Point.CARTESIAN),
        toBackboardMiddle.setConstantHeadingInterpolation(Math.toRadians(0));

        toBackboardLeft = new Path(new BezierLine(new Point(43, -35, Point.CARTESIAN), (new Point(47.5, -29, Point.CARTESIAN))));
        toBackboardLeft.setConstantHeadingInterpolation(Math.toRadians(0));

        toBackboardRight = new Path(new BezierLine(new Point(43, -35, Point.CARTESIAN), new Point(47.5, -38, Point.CARTESIAN)));
        toBackboardRight.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackMiddle = new Path(new BezierLine(new Point(-55, -21.5, Point.CARTESIAN), new Point(-57, -19, Point.CARTESIAN)));
        toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackLeftFromBB = new Path(new BezierCurve((new Point(47.25, -38, Point.CARTESIAN)), (new Point(18, -56, Point.CARTESIAN)), (new Point(-10, -61, Point.CARTESIAN)), (new Point(-54, -62, Point.CARTESIAN)), (new Point(-57, -48, Point.CARTESIAN))));
        toStackLeftFromBB.setConstantHeadingInterpolation(0);
        toStackLeftFromBB.setReversed(true);

        toStrafeAtLeftStack = new Path(new BezierLine((new Point(-58, 35, Point.CARTESIAN)), (new Point(-55, 35, Point.CARTESIAN))));

        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.RED;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        robot.follower.setStartingPose(new Pose(-39, -58, Math.toRadians(90)));

//        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//
//        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (!isStopRequested() && !isStarted()) {

            zone = pipeline.detectZone();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new followPath(CenterstageConstants.getPath(zone, toSpikeLeft, toSpikeMiddle, toSpikeRight)),
//                            new pitchToTransferPosition(),
//                            new pivotToTransferPosition(),
//                            new releaseLeftPixel(),
//                            new WaitCommand(100),
//                            new grabRightPixel(),
//                            new WaitCommand(500),
//                            new pitchToSpikePosition(),
//                            new WaitCommand(100),
//                            new pivotToSpikePosition(),
//                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
//                            new releaseRightPixel(),
//                            new WaitCommand(100),
                            new followPath(toStackMiddle),
//                            new v4BarToHeight(5),
//                            new pivotToWaitPosition(),
//                            new pitchToWaitPosition(),
//                            new outtakeCommand(),
//                            new WaitCommand(500),
////                            new ParallelRaceGroup(
//                                    new WaitCommand(500),
////                                    new WaitUntilCommand(pixels)
////                            ),
//                            new stopIntake(),
                            new followPath(toBackboard),
//                            new WaitCommand(200),
//                            new pivotToTransferPosition(),
//                            new WaitCommand(100),
//                            new pitchToTransferPosition(),
//                            new WaitCommand(300),
//                            new grabLeftPixel(),
//                            new grabRightPixel(),
//                            new intakeCommand(),
//
//                            new WaitCommand(500),
//                            new stopIntake(),

                            new WaitUntilCommand(busy),
//                            new pitchToDropPosition(),
//                            new WaitCommand(100),
//                            new pivotToDropPosition(),
//                            new WaitCommand(500),
                            new followPath(CenterstageConstants.getPath(zone, toBackboardLeft, toBackboardMiddle, toBackboardRight)),
                            new WaitUntilCommand(busy),
//                            new WaitCommand(200),
//                            new releaseLeftPixel(),
//                            new releaseRightPixel(),
//                            new WaitCommand(200),
//                            new pivotToWaitPosition(),
//                            new pitchToWaitPosition(),
//                            new WaitCommand(1000),

                            new followPath(toStackLeftFromBB),
                            new WaitUntilCommand(busy)
//                            new v4BarToHeight(5),
//                            new outtakeCommand(),
//                            new WaitUntilCommand(busy),
//                            new WaitCommand(350),
//                            new followPath(toStrafeAtLeftStack),
//                            new v4BarToHeight(4),
//                            new WaitCommand(150),
//                            new v4BarToHeight(3),
//                            new WaitCommand(150),
//
//                            new v4BarToHeight(5),
//                            new WaitCommand(150),
//                            new v4BarToHeight(4),
//                            new WaitCommand(150),
//                            new v4BarToHeight(3),
//                            new WaitCommand(150),
//
//                            new WaitUntilCommand(busy),
//                            new WaitCommand(150),
//                            new v4BarToHeight(4),
//                            new WaitCommand(500),
//                            new v4BarToHeight(3),
//                            new ParallelRaceGroup(
//                                    new WaitUntilCommand(pixels),
//                                    new WaitCommand(3500)
//                            ),
//
//                            new stopIntake(),
//                            new intakeCommand(),
//                            new WaitCommand(200),
//                            new stopIntake(),
//                            new WaitCommand(400),
//                            new pivotToTransferPosition(),
//                            new WaitCommand(200),
//                            new pitchToTransferPosition(),
//
//                            new followPath(toBackboard),
//                            new intakeCommand(),
//                            new WaitCommand(500),
//                            new stopIntake(),
//                            new grabLeftPixel(),
//                            new grabRightPixel(),
//                            new WaitUntilCommand(busy),
//                            new pitchToDropPosition(),
//                            new WaitCommand(80),
//                            new pivotToDropPosition(),
//                            new slideToRow(3),
//                            new WaitCommand(600),
//                            new releaseRightPixel(),
//                            new releaseLeftPixel(),
//
//                            new slideToRow(1),
//                            new pivotToWaitPosition(),
//                            new pitchToWaitPosition(),
//
//                            new followPath(toStackLeftFromBB),
//                            new v4BarToHeight(5),
//                            new outtakeCommand(),
//                            new WaitUntilCommand(busy),
//                            new WaitCommand(350),
//                            new followPath(toStrafeAtLeftStack),
//                            new v4BarToHeight(4),
//                            new WaitCommand(150),
//                            new v4BarToHeight(3),
//                            new WaitCommand(150),
//
//                            new v4BarToHeight(5),
//                            new WaitCommand(150),
//                            new v4BarToHeight(4),
//                            new WaitCommand(150),
//                            new v4BarToHeight(3),
//                            new WaitCommand(150),
//
//                            new WaitUntilCommand(busy),
//                            new WaitCommand(150),
//                            new v4BarToHeight(4),
//                            new WaitCommand(500),
//                            new v4BarToHeight(3),
//                            new ParallelRaceGroup(
//                                    new WaitUntilCommand(pixels),
//                                    new WaitCommand(3500)
//                            ),
//
//                            new stopIntake(),
//                            new intakeCommand(),
//                            new WaitCommand(200),
//                            new stopIntake(),
//                            new WaitCommand(400),
//                            new pivotToTransferPosition(),
//                            new WaitCommand(200),
//                            new pitchToTransferPosition(),
//
//                            new followPath(toBackboard),
//                            new intakeCommand(),
//                            new WaitCommand(500),
//                            new stopIntake(),
//                            new grabLeftPixel(),
//                            new grabRightPixel(),
//                            new WaitUntilCommand(busy),
//                            new pitchToDropPosition(),
//                            new WaitCommand(80),
//                            new pivotToDropPosition(),
//                            new slideToRow(3),
//                            new WaitCommand(600),
//                            new releaseRightPixel(),
//                            new releaseLeftPixel(),
//
//                            new slideToRow(1),
//                            new pivotToWaitPosition(),
//                            new pitchToWaitPosition()
                    )
            );


            telemetry.addLine("Robot Initialized.");
            telemetry.addData("zone", zone);
            telemetry.update();
        }


        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.periodic();
            robot.write();

            currentTime = System.nanoTime();
            loopTime = currentTime - lastTime;
            lastTime = currentTime;

            telemetry.addData("looptime", loopTime);
            telemetry.addData("is it busy",busy.getAsBoolean());

            telemetry.addData("x", robot.poseUpdater.getPose().getX());
            telemetry.addData("y", robot.poseUpdater.getPose().getY());
            telemetry.addData("heading", Math.toDegrees(robot.poseUpdater.getPose().getHeading()));
            telemetry.addData("total heading", Math.toDegrees(robot.poseUpdater.getTotalHeading()));
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

//new pivotToTransferPosition(),
//        new WaitCommand(80),
//        new pitchToTransferPosition(),
//        new WaitCommand(200),
//        new grabRightPixel(),
//        new grabLeftPixel(),
//        new followPath(toSpike),
//        new WaitUntilCommand(busy),
//        new pitchToSpikePosition(),
//        new WaitCommand(100),
//        new pivotToSpikePosition(),
//        new WaitCommand(300),
//        new releaseLeftPixel(),
//        new WaitCommand(500),
//        new ParallelCommandGroup(
//        new followPath(toBackboard),
//        new pitchToDropPosition(),
//        new pivotToDropPosition()
//        ),
//        new WaitUntilCommand(busy),
//        new WaitCommand(500),
//        new releaseRightPixel()