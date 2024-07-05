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
public class AutoRedFar extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private double currentTime=0;
    private double lastTime = 0.0;
    private double loopTime=0;
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toStackMiddle;
    private Path toStackLeftFromBB;
    private Path toStrafeAtLeftStack;
    private Path toBackboard;
    private Path toDrop;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toPark;


    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >26;

    private int zone;
    double bbDropY=-36;
    double bbDropX=45;
    @Override
    public void initialize() {


        telemetry.setMsTransmissionInterval(50);


        toSpikeMiddle = new Path(new BezierLine(new Point(-39, -58,Point.CARTESIAN), new Point(-55, -21.5,Point.CARTESIAN)));
        toSpikeMiddle.setLinearHeadingInterpolation(Math.toRadians(90),0);
        toSpikeLeft = new Path(new BezierLine(new Point(-39, -58,Point.CARTESIAN), new Point(-57, -17,Point.CARTESIAN)));
        toSpikeLeft.setConstantHeadingInterpolation(Math.toRadians(-44));
        toSpikeRight = new Path(new BezierLine(new Point(-39, -58,Point.CARTESIAN), new Point(-39.4, -34.6,Point.CARTESIAN)));
        toSpikeRight.setConstantHeadingInterpolation(Math.toRadians(0));

//        toBackboard = new Path(new BezierCurve(new Point(-57, -14,Point.CARTESIAN), new Point(-27,4,Point.CARTESIAN), new Point(30,2,Point.CARTESIAN),new Point(43,-35,Point.CARTESIAN),new Point(45, -36,Point.CARTESIAN)));
//        toBackboard.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        toBackboardMiddle = new Path(new BezierLine(new Point(43, -35, Point.CARTESIAN),new Point(45, -36,Point.CARTESIAN))); //new Point(21, 0, Point.CARTESIAN),
//        toBackboardMiddle.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        toBackboardLeft = new Path(new BezierLine(new Point(43, -35, Point.CARTESIAN),(new Point(45, -29,Point.CARTESIAN))));
//        toBackboardLeft.setConstantHeadingInterpolation(Math.toRadians(0));
//
//        toBackboardRight = new Path(new BezierLine(new Point(43, -35, Point.CARTESIAN),new Point(45,-38,Point.CARTESIAN)));
//        toBackboardRight.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackMiddle = new Path(new BezierLine(new Point(-55,-21.5,Point.CARTESIAN),new Point(-57, -19.5, Point.CARTESIAN)));
        toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackLeftFromBB = new Path(new BezierCurve((new Point(43,-35,Point.CARTESIAN)),(new Point(30,2,Point.CARTESIAN)),(new Point(-27,4,Point.CARTESIAN)),(new Point(-57, -19,Point.CARTESIAN))));
        toStackLeftFromBB.setConstantHeadingInterpolation(0);
//        toStackLeftFromBB.setReversed(true);

        toStrafeAtLeftStack = new Path(new BezierLine((new Point(-58, 35, Point.CARTESIAN)),(new Point(-55,35,Point.CARTESIAN))));

        toPark = new Path(new BezierLine(new Point(48.5,-33,Point.CARTESIAN),new Point(45,-20,Point.CARTESIAN)));
        toPark.setConstantHeadingInterpolation(0);

        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.RED;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        robot.follower.setStartingPose(new Pose(-39,-58,Math.toRadians(90)));




        while (opModeInInit()) {
            CommandScheduler.getInstance().reset();

            zone=robot.propDetectionPipeline.detectZone();

            if (zone==1){
                bbDropX=48.5;
                bbDropY=-30;
            }
            else if (zone==2) {
                bbDropX=48.5;
                bbDropY=-36;
            }
            else{
                bbDropX=48.5;
                bbDropY=-43;
            }

            toBackboard = new Path(new BezierCurve(new Point(-57, -14,Point.CARTESIAN), new Point(-27,4,Point.CARTESIAN), new Point(30,2,Point.CARTESIAN),new Point(43,-35,Point.CARTESIAN)));
            toBackboard.setConstantHeadingInterpolation(Math.toRadians(0));

            toDrop = new Path(new BezierLine(new Point(43,-35,Point.CARTESIAN),new Point(bbDropX, bbDropY,Point.CARTESIAN)));
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
                            new WaitUntilCommand(busy),
                            new ParallelRaceGroup(
                                    new WaitCommand(1000),
                                    new WaitUntilCommand(pixels)
                            ),
                            new stopIntake(),
                            new intakeCommand(),
                            new WaitCommand(400),
                            new v4BarToHeight(4),
                            new intakeCommand(),
                            new WaitCommand(400),
                            new stopIntake(),
                            //above is new
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(1000),
                            new grabLeftPixel(),
                            //below is new
                            new grabRightPixel(),
                            new followPath(toBackboard),
                            new outtakeCommand(),
                            new WaitCommand(300),
                            new stopIntake(),
                            new v4BarUp(),
                            new WaitUntilCommand(busy),
//                            new ConditionalCommand(new setRollAngle(0),new setRollAngle(Math.toRadians(180)),preload),
                            new slideToRow(2),
                            new pitchToDropPosition(),
                            new WaitCommand(250),
                            new pivotToDropPosition(),
                            new WaitCommand(500),
                            new followPath(toDrop),
                            //above is new
                            new WaitUntilCommand(busy),
                            new WaitCommand(150),
                            new releaseLeftPixel(),
                            //below is new
                            new releaseRightPixel(),
                            new slideToRow(1),
                            //above is new




                            new WaitCommand(500),
                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),

//                            new WaitCommand(100),

//                            new WaitCommand(100),
//                            new followPath(toStackLeft),
                            new WaitUntilCommand(busy),
                            new outtakeCommand(),
                            new v4BarToHeight(3),
                            new followPath(toStrafeAtLeftStack),
                            new ParallelRaceGroup(
                                    new WaitCommand(1000),
                                    new WaitUntilCommand(pixels)
                            ),
                            new stopIntake(),
                            new intakeCommand(),
                            new WaitCommand(400),
                            new v4BarToHeight(1),
                            new outtakeCommand(),
                            new WaitCommand(400),
                            new stopIntake(),
                            //above is new
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(1000),
                            new grabLeftPixel(),
                            //below is new
                            new grabRightPixel(),
                            new followPath(toBackboard),
                            new intakeCommand(),
                            new WaitCommand(300),
                            new stopIntake(),
                            new v4BarUp(),
                            new WaitUntilCommand(busy),
                            //                            new ConditionalCommand(new setRollAngle(0),new setRollAngle(Math.toRadians(180)),preload),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(250),
                            new pivotToDropPosition(),
                            new WaitCommand(500),
                            new followPath(toDrop),
                            //above is new
                            new WaitUntilCommand(busy),
                            new WaitCommand(150),
                            new releaseLeftPixel(),
                            //below is new
                            new releaseRightPixel(),
                            //above is new
                            new WaitCommand(150),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new v4BarUp()
                    )
            );



            telemetry.addLine("Robot Initialized.");
            telemetry.addData("zone", zone);
            telemetry.update();
        }

    }

    @Override
    public void run() {
        robot.stopCameraStream();
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();

    }
}
