package org.firstinspires.ftc.teamcode.opmode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.*;
import java.util.function.BooleanSupplier;


@TeleOp
public class AutoRedClose extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Path toSpike;
    private Path toStackRight;
    private Path toStackMiddle;
    private Path toStrafeAtStackRight;
    private Path toStrafeAtStackMiddle;
    private Path toBackboard;
    private Path toBackboardFromStack;
    private Path toPark;
    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >26;
//    public enum RedDropClose{
//        left,
//        middle,
//        right
//    }
//    public RedDropClose side;
    VisionPortal visionPortal;
    public PropDetectionPipeline pipeline;
    private int zone;
    private Location randomization;

    @Override
    public void initialize() {
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        pipeline = new PropDetectionPipeline(0.3, 0.7, 0.65);
        startCamera();
        zone= pipeline.detectZone();




//        CenterstageConstants.RANDOMIZATION=Location.

        telemetry.setMsTransmissionInterval(50);
//        robot.follower.setStartingPose();
        toSpike = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(21, 0,Point.CARTESIAN)));
        toSpike.setConstantHeadingInterpolation(Math.toRadians(0));

        toBackboard = new Path(new BezierCurve(new Point(21, 0, Point.CARTESIAN), new Point(23, -36.25,Point.CARTESIAN)));
        toBackboard.setConstantHeadingInterpolation(Math.toRadians(-90));

        toStackRight = new Path(new BezierCurve(new Point(23, -36.25, Point.CARTESIAN), new Point(56.6, -33,Point.CARTESIAN),new Point(65, 0,Point.CARTESIAN),(new Point(48.17, 68.7, Point.CARTESIAN))));
        toStackRight.setConstantHeadingInterpolation(Math.toRadians(-85));
        toStackRight.setReversed(true);
        toStrafeAtStackRight = new Path(new BezierLine((new Point(48.17,68.2,Point.CARTESIAN)),(new Point(45.6,68.5,Point.CARTESIAN))));
        toStrafeAtStackRight.setConstantHeadingInterpolation(Math.toRadians(-85));

        toStackMiddle = new Path(new BezierCurve(new Point(23, -36, Point.CARTESIAN), new Point(56.6, -33,Point.CARTESIAN),new Point(65, 0,Point.CARTESIAN),(new Point(34, 68, Point.CARTESIAN))));
        toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));
        toStackMiddle.setReversed(true);
        toStrafeAtStackMiddle = new Path(new BezierLine((new Point(34,67.5,Point.CARTESIAN)),(new Point(30 ,67.5,Point.CARTESIAN))));
        toStrafeAtStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));

        toBackboardFromStack = new Path(new BezierCurve((new Point(39.3, 68.65, Point.CARTESIAN)),(new Point(65, 0,Point.CARTESIAN)),(new Point(56.6, -33,Point.CARTESIAN)),(new Point(23, -37.5,Point.CARTESIAN))));
        toBackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(-90));
        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new followPath(toSpike),
                        new pitchToTransferPosition(),
                        new pivotToTransferPosition(),
                        new WaitCommand(100),
                        new grabRightPixel(),
                        new grabLeftPixel(),
                        new WaitCommand(500),
                        new pitchToSpikePosition(),
                        new WaitCommand(100),
                        new pivotToSpikePosition(),
                        new WaitCommand(300),
                        new WaitUntilCommand(busy),
                        new releaseLeftPixel(),
                        new WaitCommand(100),
                        new followPath(toBackboard),
                        new pitchToDropPosition(),
                        new pivotToDropPosition(),

                        new WaitUntilCommand(busy),
                        new WaitCommand(500),
                        new releaseRightPixel(),
                        new pivotToWaitPosition(),
                        new pitchToWaitPosition(),

                        new followPath(toStackRight),
                        new v4BarToHeight(5),
                        new outtakeCommand(),
                        new WaitUntilCommand(busy),
                        new WaitCommand(350),
                        new followPath(toStrafeAtStackRight),
                        new v4BarToHeight(4),
                        new WaitCommand(150),
                        new v4BarToHeight(3),
                        new WaitCommand(150),

                        new v4BarToHeight(5),
                        new WaitCommand(150),
                        new v4BarToHeight(4),
                        new WaitCommand(150),
                        new v4BarToHeight(3),
                        new WaitCommand(150),

                        new WaitUntilCommand(busy),
                        new WaitCommand(150),
                        new v4BarToHeight(4),
                        new WaitUntilCommand(pixels),

                        new stopIntake(),
                     //   new WaitUntilCommand(pixels),
                        new intakeCommand(),
                        new WaitCommand(200),
                        new stopIntake(),
                        new WaitCommand(400),
                        new pivotToTransferPosition(),
                        new WaitCommand(200),
                        new pitchToTransferPosition(),

                        new followPath(toBackboardFromStack),
                        new intakeCommand(),
                        new WaitCommand(500),
                        new stopIntake(),
                        new grabLeftPixel(),
                        new grabRightPixel(),
                        new WaitUntilCommand(busy),
                        new pitchToDropPosition(),
                        new WaitCommand(80),
                        new pivotToDropPosition(),
                        new slideToRow(3),
                        new WaitCommand(600),
                        new releaseRightPixel(),
                        new releaseLeftPixel(),

                        new slideToRow(1),
                        new pivotToWaitPosition(),
                        new pitchToWaitPosition(),

                        new followPath(toStackMiddle),
                        new v4BarToHeight(5),
                        new outtakeCommand(),
                        new WaitUntilCommand(busy),
                        new WaitCommand(750),
                        new followPath(toStrafeAtStackMiddle),
                        new v4BarToHeight(4),
                        new WaitCommand(50),
                        new v4BarToHeight(3),
                        new WaitCommand(50),

                        new v4BarToHeight(5),
                        new WaitCommand(50),
                        new v4BarToHeight(4),
                        new WaitCommand(50),
                        new v4BarToHeight(3),
                        new WaitCommand(50),

                        new WaitUntilCommand(busy),
                        new WaitCommand(50),
                        new v4BarToHeight(4),
                        new WaitUntilCommand(busy),

                        new intakeCommand(),
                        new WaitCommand(200),
                        new stopIntake(),
                        new WaitCommand(400),
                        new pivotToTransferPosition(),
                        new WaitCommand(200),
                        new pitchToTransferPosition(),

                        new followPath(toBackboardFromStack),
                        new WaitCommand(500),
                        new grabLeftPixel(),
                        new grabRightPixel(),
                        new WaitUntilCommand(busy),
                        new pitchToDropPosition(),
                        new WaitCommand(80),
                        new pivotToDropPosition(),
                        new slideToRow(3),
                        new WaitCommand(600),
                        new releaseRightPixel(),
                        new releaseLeftPixel()

                )
        );

        while (opModeInInit()) {
            if (zone==1) randomization=Location.LEFT;
            else if (zone==2) randomization=Location.MIDDLE;
            else randomization=Location.RIGHT;
            telemetry.addLine("Robot Initialized.");
            telemetry.addData("zone", pipeline.detectZone());
            telemetry.update();
        }

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();


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