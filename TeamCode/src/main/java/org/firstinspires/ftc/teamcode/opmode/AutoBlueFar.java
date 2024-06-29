package org.firstinspires.ftc.teamcode.opmode;

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
import org.firstinspires.ftc.teamcode.common.pathing.localization.Pose;
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


@Autonomous
public class AutoBlueFar extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toStackLeft;
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

    VisionPortal visionPortal;
    public PropDetectionPipeline pipeline;
    private int zone;
    private Location randomization;

    @Override
    public void initialize() {
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        pipeline = new PropDetectionPipeline(0.3, 0.7, 0.65);
        startCamera();
        zone=pipeline.detectZone();


        telemetry.setMsTransmissionInterval(50);


        toSpikeMiddle = new Path(new BezierLine(new Point(0, 0,Point.CARTESIAN), new Point(0, 0,Point.CARTESIAN)));
        toSpikeMiddle.setConstantHeadingInterpolation(Math.toRadians(0));
        toSpikeLeft = new Path(new BezierCurve(new Point(0, 0,Point.CARTESIAN), new Point(0, 0,Point.CARTESIAN)));
        toSpikeLeft.setConstantHeadingInterpolation(Math.toRadians(90));
        toSpikeRight = new Path(new BezierCurve(new Point(0, 0,Point.CARTESIAN), new Point(0, 0,Point.CARTESIAN)));
        toSpikeRight.setConstantHeadingInterpolation(Math.toRadians(90));

        toBackboard = new Path(new BezierCurve(new Point(0, 0,Point.CARTESIAN))); //new Point(21, 0, Point.CARTESIAN),
        toBackboard.setConstantHeadingInterpolation(Math.toRadians(-90));

        toStackRight = new Path(new BezierCurve(new Point(0, 0, Point.CARTESIAN), new Point(0, 0,Point.CARTESIAN),new Point(0, 0,Point.CARTESIAN),(new Point(0, 0, Point.CARTESIAN))));
        toStackRight.setConstantHeadingInterpolation(Math.toRadians(-85));
        toStackRight.setReversed(true);
        toStrafeAtStackRight = new Path(new BezierLine((new Point(0,0,Point.CARTESIAN)),(new Point(0,0,Point.CARTESIAN))));
        toStrafeAtStackRight.setConstantHeadingInterpolation(Math.toRadians(-85));

        toStackMiddle = new Path(new BezierCurve(new Point(0, 0, Point.CARTESIAN), new Point(0, 0,Point.CARTESIAN),new Point(0, 0,Point.CARTESIAN),(new Point(0, 0, Point.CARTESIAN))));
        toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));
        toStackMiddle.setReversed(true);
        toStrafeAtStackMiddle = new Path(new BezierLine((new Point(0,0,Point.CARTESIAN)),(new Point(0 ,0,Point.CARTESIAN))));
        toStrafeAtStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));

        toStackRight = new Path(new BezierCurve(new Point(0, 0, Point.CARTESIAN),new Point(0, 0,Point.CARTESIAN)));
        toStackRight.setConstantHeadingInterpolation(Math.toRadians(-85));
        toStackRight.setReversed(true);

        toBackboardFromStack = new Path(new BezierCurve((new Point(0, 0, Point.CARTESIAN)),(new Point(0, 0,Point.CARTESIAN)),(new Point(0, -0,Point.CARTESIAN)),(new Point(0, 0,Point.CARTESIAN))));
        toBackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(-90));
        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);




        while (opModeInInit()) {

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new followPath(CenterstageConstants.getPath(zone, toSpikeLeft,toSpikeMiddle,toSpikeRight)),
                            new pitchToTransferPosition(),
                            new pivotToTransferPosition(),
                            new WaitCommand(100),
                            new grabRightPixel(),
                            new WaitCommand(500),
                            new pitchToSpikePosition(),
                            new WaitCommand(100),
                            new pivotToSpikePosition(),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new releaseLeftPixel(),
                            new WaitCommand(100),
                            new followPath(toStackLeft),
                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),
                            new outtakeCommand(),
                            new v4BarToHeight(5),
                            new WaitCommand(1000),
                            new v4BarToHeight(4),
                            new ParallelRaceGroup(
                                    new WaitUntilCommand(pixels),
                                    new WaitCommand(4000)
                            ),
                            new stopIntake(),
                            new WaitCommand(200),
                            new pivotToTransferPosition(),
                            new WaitCommand(500),
                            new pitchToTransferPosition(),
                            new WaitCommand(500),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new intakeCommand(),

                            new WaitUntilCommand(busy),
                            new WaitCommand(500),
                            new stopIntake(),
                            new followPath(toBackboardFromStack),
                            new WaitUntilCommand(busy),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitCommand(500),
                            new releaseLeftPixel(),
                            new releaseRightPixel(),
                            new WaitCommand(200),
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
                            new WaitCommand(500),
                            new v4BarToHeight(3),
                            new ParallelRaceGroup(
                                    new WaitUntilCommand(pixels),
                                    new WaitCommand(3500)
                            ),

                            new stopIntake(),
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
                            new WaitCommand(350),
                            new followPath(toStrafeAtStackMiddle),
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
                            new WaitCommand(500),
                            new v4BarToHeight(3),
                            new ParallelRaceGroup(
                                    new WaitUntilCommand(pixels),
                                    new WaitCommand(3500)
                            ),

                            new stopIntake(),
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
                            new pitchToWaitPosition()
                    )
            );



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