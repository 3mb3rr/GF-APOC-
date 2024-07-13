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
import org.firstinspires.ftc.teamcode.common.commands.interruptFollower;
import org.firstinspires.ftc.teamcode.common.pathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.*;
import java.util.function.BooleanSupplier;


@Autonomous
public class AutoBlueFarRealLotus extends CommandOpMode {
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
    private Path toBackboard;
    private Path toBackBoardTest;
    private Path toBackFromMid;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toPark;
    private Path toBackboardfromstack;
    private Path toBackboardSecondTime;
    private Path TostackSecondTime;
    private Path TostrafeSecondTime;


    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >29;
    private BooleanSupplier preload = () -> robot.preloadDetectionPipeline.getPreloadedZone() == CenterstageConstants.PRELOAD;
    private BooleanSupplier pastCenter = () -> robot.follower.getPose().getX()>25;
    private int zone;
    double bbDropY;
    double bbDropX;
    double spikeDropX;
    double spikeDropY;
    long wait = 0;
    int wrist = 0;
    double whiteDropX=44;
    double whiteDropY=35;
    @Override
    public void initialize() {


        telemetry.setMsTransmissionInterval(50);




        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.BLUE;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        robot.follower.setStartingPose(new Pose(-39,58,Math.toRadians(-90)));

        while (opModeInInit()) {

            zone=robot.propDetectionPipeline.detectZone();

            if (zone==1){
                spikeDropX=-41;
                spikeDropY=33;
                bbDropX=44;
                bbDropY=42;
                wait = 1000;
                wrist=0;
            }
            else if (zone==2) {
                spikeDropX=-55;
                spikeDropY=21.5;
                bbDropX=44;
                bbDropY=37;
                wait = 250;
                wrist=0;
            }
            else{
                spikeDropX=-57;
                spikeDropY=17;
                bbDropX=44;
                bbDropY=32; //48
                wait = 750;
                wrist=0;
            }

            toSpikeMiddle = new Path(new BezierLine(new Point(-39, 56,Point.CARTESIAN), new Point(spikeDropX, spikeDropY,Point.CARTESIAN)));
            toSpikeMiddle.setLinearHeadingInterpolation(Math.toRadians(90),0);
            toSpikeLeft = new Path(new BezierLine(new Point(-39, 56,Point.CARTESIAN), new Point(spikeDropX, spikeDropY,Point.CARTESIAN)));
            toSpikeLeft.setConstantHeadingInterpolation(Math.toRadians(0));
            toSpikeRight = new Path(new BezierLine(new Point(-39, 56,Point.CARTESIAN), new Point(spikeDropX, spikeDropY,Point.CARTESIAN)));
            toSpikeRight.setConstantHeadingInterpolation(Math.toRadians(44));

            toStackMiddle = new Path(new BezierLine(new Point(spikeDropX,spikeDropY,Point.CARTESIAN),new Point(-58.3, 19, Point.CARTESIAN)));
            toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(0));

            toBackboard = new Path(new BezierCurve(
                    new Point(-58.3, 19,Point.CARTESIAN),
                    new Point(-51,6,Point.CARTESIAN),
                    new Point(-27,2,Point.CARTESIAN),
                    new Point(30,5,Point.CARTESIAN),
                    new Point(43,14 ,Point.CARTESIAN),
                    new Point(bbDropX, bbDropY,Point.CARTESIAN)));
            toBackboard.setConstantHeadingInterpolation(Math.toRadians(0));

            toStackMFromBB = new Path(new BezierCurve(
                    (new Point(bbDropX,bbDropY,Point.CARTESIAN)),
                    (new Point(30,4,Point.CARTESIAN)),
                    (new Point(-27,4,Point.CARTESIAN)),
                    (new Point(-57, 15,Point.CARTESIAN))));
            toStackMFromBB.setConstantHeadingInterpolation(0);
//        toStackMFromBB.setReversed(true);

            toStrafeAtMStack = new Path(new BezierLine(
                    (new Point(-57, 15, Point.CARTESIAN)),
                    (new Point(-59,24,Point.CARTESIAN))));
            toStrafeAtMStack.setConstantHeadingInterpolation(0);


            toBackboardfromstack = new Path(new BezierCurve(
                    new Point(-59 , 24,Point.CARTESIAN),
                    new Point(-51,6,Point.CARTESIAN),
                    new Point(-27,4,Point.CARTESIAN),
                    new Point(30,5,Point.CARTESIAN),
                    new Point(43,14 ,Point.CARTESIAN),
                    new Point(whiteDropX ,whiteDropY , Point.CARTESIAN)));
            toBackboardfromstack.setConstantHeadingInterpolation(Math.toRadians(-2));

            TostackSecondTime = new Path(new BezierCurve(
                    (new Point(whiteDropX,whiteDropY,Point.CARTESIAN)),
                    (new Point(30,4,Point.CARTESIAN)),
                    //(new Point(10,4,Point.CARTESIAN)),
                    (new Point(-27,4,Point.CARTESIAN)),
                    //(new Point(-36,4,Point.CARTESIAN)),
                    (new Point(-57 , 7,Point.CARTESIAN))));
            TostackSecondTime.setConstantHeadingInterpolation(0);

            TostrafeSecondTime = new Path(new BezierLine(
                    (new Point(-57, 7, Point.CARTESIAN)),
                    (new Point(-59,15,Point.CARTESIAN))));
            TostrafeSecondTime.setConstantHeadingInterpolation(0);


            toBackboardSecondTime =  new Path(new BezierCurve(
                    new Point(-59, 15,Point.CARTESIAN),
                    new Point(-51,6,Point.CARTESIAN),
                    new Point(-27,4,Point.CARTESIAN),
                    new Point(32,4,Point.CARTESIAN),
                    new Point(43,14 ,Point.CARTESIAN),
                    new Point(whiteDropX ,whiteDropY , Point.CARTESIAN)));
            toBackboardSecondTime.setConstantHeadingInterpolation(0);

//            toBackBoardTest = new Path(new BezierCurve(new Point(-57, -14,Point.CARTESIAN), new Point(-51,-6,Point.CARTESIAN),new Point(-27,-2,Point.CARTESIAN), new Point(30,-5,Point.CARTESIAN),new Point(43,-14 ,Point.CARTESIAN),new Point(bbDropX, bbDropY,Point.CARTESIAN)));
//            toBackBoardTest.setConstantHeadingInterpolation(Math.toRadians(0));
//            toBackFromMid = new Path(new BezierLine(new Point(30,-5,Point.CARTESIAN),new Point(bbDropX, bbDropY,Point.CARTESIAN)));
//            toBackFromMid.setConstantHeadingInterpolation(Math.toRadians(0));


            CommandScheduler.getInstance().reset();


            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new followPath(CenterstageConstants.getPath(zone, toSpikeLeft,toSpikeMiddle,toSpikeRight)),
                                    new SequentialCommandGroup(
                                            new pitchToTransferPosition(),
                                            new pivotToTransferPosition(),
                                            new WaitCommand(100),
                                            new releaseRightPixel(),
                                            new grabLeftPixel(),
                                            new WaitCommand(wait),
                                            new pitchToSpikePosition(),
                                            new WaitCommand(100),
                                            new pivotToSpikePosition(),
                                            new WaitCommand(300)
                                    ),
                                    new WaitUntilCommand(busy)
                            ),
                            new WaitCommand(200),
                            new releaseLeftPixel(),
                            new WaitCommand(100),
                            //below is new
                            new v4BarToHeight(5),
                            new outtakeCommand(), // make intake
                            new ParallelCommandGroup(
                                    new followPath(toStackMiddle),
                                    new ParallelRaceGroup(
                                            new WaitCommand(1500),
                                            new WaitUntilCommand(pixels)
                                    )
                            ),
                            new WaitCommand(300),
                            new stopIntake(),
                            new intakeCommand(), //make outtake
                            new WaitCommand(250),
                            new stopIntake(),
                            //above is new
//                            new pivotToTransferPosition(),
//                            new pitchToTransferPosition(),
//                            new WaitCommand(400),
//                            new grabLeftPixel(),
//                            //below is new
//                            new grabRightPixel()
                            new WaitCommand(3000),
                            new ParallelCommandGroup(
                                    new followPath(toBackboard),
                                    new SequentialCommandGroup(
                                            new pivotToTransferPosition(),
                                            new pitchToTransferPosition(),
                                            new WaitCommand(400),
                                            new grabLeftPixel(),
                                            new grabRightPixel()
                                            //below is new
                                    )
//                                    new WaitUntilCommand(busy)
                            ),
                            new WaitUntilCommand(pastCenter),
                            new slideToRow(2),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new setRollAngle(Math.toRadians(wrist)),
                            new WaitUntilCommand(busy),
                            new WaitCommand(100),
                            new releaseLeftPixel(),
                            new releaseRightPixel(),
                            new WaitCommand(150),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new setRollAngle(Math.toRadians(0)),

                            new followPath(toStackMFromBB),
                            new v4BarToHeight(3),
                            new WaitUntilCommand(busy),
                            new followPath(toStrafeAtMStack),
                            new v4BarToHeight(1),
                            new WaitUntilCommand(busy),
                            new v4BarToHeight(4),
                            new outtakeCommand(),
                            new WaitCommand(100),
                            new v4BarToHeight(2),
                            new WaitCommand(200),
                            new v4BarToHeight(1),
                            new WaitCommand(300),
                            new followPath(toBackboardfromstack),
                            new intakeCommand(),
                            new WaitCommand(50),
                            new outtakeCommand(),
                            new WaitCommand(200),
                            new intakeCommand(),
                            new WaitCommand(100),
                            new stopIntake(),
                            new WaitCommand(100),
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(400),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new WaitUntilCommand(pastCenter),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitUntilCommand(busy),
                            new WaitCommand(100),
                            new releaseLeftPixel(),
                            new releaseRightPixel(),
                            new WaitCommand(300),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
//                            new intakeAutoState(),
//                            new followPath(TostackSecondTime),
//                            new WaitUntilCommand(busy),
//                            new outtakeCommand(),
//                            new v4BarToHeight(1),
//                            new followPath(TostrafeSecondTime),
//                            new WaitCommand(200),
//                            new v4BarToHeight(3),
//                            new WaitCommand(200),
//                            new v4BarToHeight(2),
//                            new WaitCommand(100),
//                            new v4BarToHeight(1),
//                            new WaitCommand(200),
//                            new WaitUntilCommand(busy),
//                            new followPath(toBackboardSecondTime),
//                            new intakeCommand(),
//                            new WaitCommand(200),
//                            new outtakeCommand(),
//                            new WaitCommand(300),
//                            new intakeCommand(),
//                            new WaitCommand(300),
//                            new stopIntake(),
//                            new WaitCommand(400),
//                            new pivotToTransferPosition(),
//                            new pitchToTransferPosition(),
//                            new WaitCommand(700),
//                            new grabLeftPixel(),
//                            new grabRightPixel(),
//                            new WaitUntilCommand(pastCenter),
//                            new slideToRow(3),
//                            new pitchToDropPosition(),
//                            new WaitCommand(100),
//                            new pivotToDropPosition(),
//                            new WaitUntilCommand(busy),
//                            new WaitCommand(100),
//                            new releaseLeftPixel(),
//                            new releaseRightPixel(),
//                            new WaitCommand(200),
//                            new slideToRow(1),
//                            new pitchToWaitPosition(),
//                            new pivotToWaitPosition(),
//                            new WaitCommand(200),
                            new intakeAutoState()
                    )
            );

            //above is new
//                            new followPath(toBackboard),
//                            new WaitUntilCommand(busy),
//                            new pitchToDropPosition(),
//                            new WaitCommand(250),
//                            new pivotToDropPosition(),
//                            new WaitCommand(1000),
//                            new releaseLeftPixel(),
//                            //below is new
//                            new releaseRightPixel()
            //above is new


//                            new WaitUntilCommand(busy),
//                            new WaitCommand(500),
//                            new releaseLeftPixel(),
//                            new pivotToWaitPosition(),
//                            new pitchToWaitPosition(),
//
////                            new WaitCommand(100),
//
////                            new WaitCommand(100),
//                            new followPath(toBackboardMiddle),
//                            new WaitUntilCommand(busy),
//                            new followPath(toStackMFromBB),
//                            new WaitUntilCommand(busy)




            telemetry.addLine("Robot Initialized.");
            telemetry.addData("zone", zone);
            telemetry.update();
        }

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.stopCameraStream();
        robot.read();
        robot.periodic();
        robot.write();
        telemetry.addData("is it busy", busy.getAsBoolean());
        telemetry.addData("zone", zone);
        telemetry.update();
    }
}