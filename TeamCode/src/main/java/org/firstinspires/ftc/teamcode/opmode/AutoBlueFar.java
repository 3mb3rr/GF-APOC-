package org.firstinspires.ftc.teamcode.opmode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.AprilTagPose;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToSpikePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToSpikePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pivotToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.setRollAngle;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.slideToRow;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.followPath;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeAutoState;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeToHang;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.outtakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.stopIntake;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.v4BarToHeight;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.v4BarUp;
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
public class AutoBlueFar extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private double currentTime=0;
    private double lastTime = 0.0;
    private double loopTime=0;
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toStackMiddle;
    private Path toStackMFromBB;
    private Path toStrafeAtMStack;
    private Path toStackRFromBB;
    private Path toStrafeAtRStack;
    private Path toBackboard;
    private Path toDrop;
    private Path toDropWhites;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toPark;


    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier preload = () -> robot.preloadDetectionPipeline.getPreloadedZone()==Location.LEFT;
    private BooleanSupplier time = () -> robot.getTimeSec() >26;

    private int zone;
    double bbDropY=35;
    double bbDropX=45;
    @Override
    public void initialize() {


        telemetry.setMsTransmissionInterval(50);


        toSpikeMiddle = new Path(new BezierLine(new Point(-39, 58,Point.CARTESIAN), new Point(-57.5, 23.5,Point.CARTESIAN)));
        toSpikeMiddle.setLinearHeadingInterpolation(Math.toRadians(-90),0);
        toSpikeRight = new Path(new BezierLine(new Point(-39, 58,Point.CARTESIAN), new Point(-57, 17,Point.CARTESIAN)));
        toSpikeRight.setConstantHeadingInterpolation(Math.toRadians(44));
        toSpikeLeft = new Path(new BezierCurve(new Point(-39, 58,Point.CARTESIAN), new Point(-50,40,Point.CARTESIAN),new Point(-42, 33,Point.CARTESIAN)));
        toSpikeLeft.setLinearHeadingInterpolation(Math.toRadians(-90),Math.toRadians(0));

        toBackboard = new Path(new BezierCurve(new Point(-57, 14,Point.CARTESIAN), new Point(-44,1,Point.CARTESIAN), new Point(30,3,Point.CARTESIAN),new Point(44,35.6,Point.CARTESIAN)));
        toBackboard.setConstantHeadingInterpolation(Math.toRadians(0));

//        toBackboardMiddle = new Path(new BezierLine(new Point(43, -35, Point.CARTESIAN),new Point(45, -36,Point.CARTESIAN))); //new Point(21, 0, Point.CARTESIAN),
//        toBackboardMiddle.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        toBackboardLeft = new Path(new BezierLine(new Point(43, -35, Point.CARTESIAN),(new Point(45, -29,Point.CARTESIAN))));
//        toBackboardLeft.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        toBackboardRight = new Path(new BezierLine(new Point(43, -35, Point.CARTESIAN),new Point(45,-38,Point.CARTESIAN)));
//        toBackboardRight.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackMiddle = new Path(new BezierLine(new Point(-55,24,Point.CARTESIAN),new Point(-58.3, 20, Point.CARTESIAN)));
        toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackMFromBB = new Path(new BezierCurve((new Point(43,36,Point.CARTESIAN)),(new Point(30,1,Point.CARTESIAN)),(new Point(-27,4,Point.CARTESIAN)),(new Point(-57.4, 13,Point.CARTESIAN))));
        toStackMFromBB.setConstantHeadingInterpolation(0);

        toStrafeAtMStack = new Path(new BezierLine((new Point(-58.8, 13, Point.CARTESIAN)),(new Point(-58.8,19,Point.CARTESIAN))));
        toStrafeAtMStack.setConstantHeadingInterpolation(0);

        toStackRFromBB = new Path(new BezierCurve((new Point(43,36,Point.CARTESIAN)),(new Point(30,1,Point.CARTESIAN)),(new Point(-27,4,Point.CARTESIAN)),(new Point(-56.6, 0,Point.CARTESIAN))));
        toStackRFromBB.setConstantHeadingInterpolation(0);

        toStrafeAtRStack = new Path(new BezierLine((new Point(-58.1, 0, Point.CARTESIAN)),(new Point(-58.7,9,Point.CARTESIAN))));
        toStrafeAtRStack.setConstantHeadingInterpolation(0);

        toDropWhites = new Path(new BezierLine(new Point(45,33,Point.CARTESIAN),new Point(45.25,34,Point.CARTESIAN)));
        toDropWhites.setConstantHeadingInterpolation(Math.toRadians(0));


        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.BLUE;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        robot.follower.setStartingPose(new Pose(-39,58,Math.toRadians(-90)));




        while (opModeInInit()) {
            CommandScheduler.getInstance().reset();

            zone=robot.propDetectionPipeline.detectZone();

            if (zone==1){
                bbDropX=46.5;
                bbDropY=39;
            }
            else if (zone==2) {
                bbDropX=46.5;
                bbDropY=34.5;
            }
            else{
                bbDropX=46.5;
                bbDropY=32;
            }



            toDrop = new Path(new BezierLine(new Point(45,33,Point.CARTESIAN),new Point(bbDropX,bbDropY,Point.CARTESIAN)));
            toDrop.setConstantHeadingInterpolation(Math.toRadians(0));


            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new followPath(CenterstageConstants.getPath(zone, toSpikeLeft,toSpikeMiddle,toSpikeRight)),
                            new releaseRightPixel(),
                            new pitchToSpikePosition(),
                            new WaitCommand(100),
                            new pivotToSpikePosition(),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new releaseLeftPixel(),
                            new WaitCommand(100),
                            //below is new
                            new v4BarToHeight(5),
                            new outtakeCommand(),
                            new followPath(toStackMiddle),
                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),
                            new WaitUntilCommand(busy),
                            new ParallelRaceGroup(
                                    new WaitCommand(800),
                                    new WaitUntilCommand(pixels)
                            ),
                            new stopIntake(),
                            new followPath(toBackboard),
                            new WaitCommand(150),
                            //above is new
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(550),
                            new grabLeftPixel(),
                            //below is new
                            new grabRightPixel(),
                            new WaitCommand(500),
                            new intakeCommand(),
                            new WaitCommand(600),
                            new stopIntake(),
                            new intakeAutoState(),
                            new WaitCommand(1000), //was wait until busy
//                            new ConditionalCommand(new setRollAngle(0),new setRollAngle(Math.toRadians(180)),preload),
                            new slideToRow(1),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitCommand(100),
                            new followPath(toDrop),
                            //above is new
                            new WaitUntilCommand(busy),
                            new WaitCommand(50),
                            new releaseLeftPixel(),
                            //below is new
                            new releaseRightPixel(),
                            new WaitCommand(200),
                            new slideToRow(1),
                            //above is new





                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),

//                            new WaitCommand(100),

//                            new WaitCommand(100),
                            new followPath(toStackMFromBB),
                            new WaitUntilCommand(busy),
                            new outtakeCommand(),
                            new v4BarToHeight(1),
                            new followPath(toStrafeAtMStack),
                            new WaitUntilCommand(busy),
                            new v4BarToHeight(5),
                            new WaitCommand(200),
                            new v4BarToHeight(3),
                            new WaitCommand(200),
                            new v4BarToHeight(2),
                            new WaitCommand(300),
                            new ParallelRaceGroup(
                                    new WaitCommand(200),
                                    new WaitUntilCommand(pixels)
                            ),
                            new stopIntake(),
                            new v4BarToHeight(5),
                            new followPath(toBackboard),
                            new WaitCommand(500),
                            new intakeCommand(),
                            new WaitCommand(100),
                            new outtakeCommand(),
                            new WaitCommand(300),
                            new intakeCommand(),
                            new WaitCommand(100),
                            new outtakeCommand(),
                            new WaitCommand(200),
                            new stopIntake(),
                            new WaitCommand(200),
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(300),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new intakeAutoState(),
                            new WaitCommand(1200), //was wait until busy
                            //                            new ConditionalCommand(new setRollAngle(0),new setRollAngle(Math.toRadians(180)),preload),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitCommand(100),
                            new followPath(toDropWhites),
                            //above is new
                            new WaitUntilCommand(busy),
                            new WaitCommand(50),
                            new releaseLeftPixel(),
                            //below is new
                            new releaseRightPixel(),
                            //above is new
                            new WaitCommand(150),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new slideToRow(1),





                            new followPath(toStackRFromBB),
                            new WaitUntilCommand(busy),
                            new outtakeCommand(),
                            new v4BarToHeight(3),
                            new followPath(toStrafeAtRStack),
                            new WaitUntilCommand(busy),
                            new v4BarToHeight(5),
                            new WaitCommand(200),
                            new v4BarToHeight(3),
                            new WaitCommand(200),
                            new v4BarToHeight(2),
                            new WaitCommand(300),
                            new ParallelRaceGroup(
                                    new WaitCommand(200),
                                    new WaitUntilCommand(pixels)
                            ),
                            new stopIntake(),
                            new v4BarToHeight(5),
                            //above is new

                            new followPath(toBackboard),
                            new WaitCommand(500),
                            new intakeCommand(),
                            new WaitCommand(100),
                            new outtakeCommand(),
                            new WaitCommand(300),
                            new intakeCommand(),
                            new WaitCommand(100),
                            new outtakeCommand(),
                            new WaitCommand(200),
                            new stopIntake(),
                            new WaitCommand(200),
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(500),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new intakeAutoState(),
                            new WaitCommand(1000), //was wait until busy
                            //                            new ConditionalCommand(new setRollAngle(0),new setRollAngle(Math.toRadians(180)),preload),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new followPath(toDropWhites),
                            //above is new
                            new WaitUntilCommand(busy),
                            new WaitCommand(50),
                            new releaseLeftPixel(),
                            //below is new
                            new releaseRightPixel(),
                            //above is new
                            new WaitCommand(150),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),

                            new intakeAutoState()
                    )
            );



            telemetry.addLine("Robot Initialized.");
            telemetry.addData("zone", zone);
            telemetry.update();
        }

    }

    @Override
    public void run() {
        robot.poseUpdater.update();
        robot.stopCameraStream();
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();

        telemetry.addData("x", robot.poseUpdater.getPose().getX());
        telemetry.addData("y", robot.poseUpdater.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(robot.poseUpdater.getPose().getHeading()));
        telemetry.addData("total heading", Math.toDegrees(robot.poseUpdater.getTotalHeading()));
        telemetry.update();
    }
}
