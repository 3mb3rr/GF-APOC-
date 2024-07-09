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
import org.firstinspires.ftc.teamcode.common.commands.armCommands.setRollAngle;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.slideToRow;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.followPath;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeAutoState;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.outtakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.stopIntake;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.v4BarToHeight;

import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.function.BooleanSupplier;


@Autonomous
public class AutoBlueClose extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toStackLeft;
    private Path toStackLeft2;
    private Path toStackMiddle;
    private Path toStrafeAtStackLeft;
    private Path toStrafeAtStackLeft2;
    private Path toStrafeAtStackMiddle;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toBackboardFromStack;
    private Path toPark;
    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >26;
    private BooleanSupplier pastCenter = () -> robot.follower.getPose().getY()>5;

    private int zone;
    @Override
    public void initialize() {


        telemetry.setMsTransmissionInterval(50);

        toSpikeMiddle = new Path(new BezierLine(new Point(0, 0,Point.CARTESIAN),new Point(33.5, 17.5,Point.CARTESIAN)));
        toSpikeMiddle.setLinearHeadingInterpolation(0,Math.toRadians(-90));

        toSpikeLeft = new Path(new BezierCurve(new Point(0, 0,Point.CARTESIAN), new Point(13,13,Point.CARTESIAN), new Point(28.5,23,Point.CARTESIAN)));
        toSpikeLeft.setLinearHeadingInterpolation(0,Math.toRadians(-90));

        toSpikeRight = new Path(new BezierCurve(new Point(0, 0,Point.CARTESIAN), new Point(16,6,Point.CARTESIAN),new Point(22, 3,Point.CARTESIAN)));
        toSpikeRight.setLinearHeadingInterpolation(0,Math.toRadians(-90));

        toBackboardMiddle = new Path(new BezierLine(new Point(21, 0, Point.CARTESIAN),new Point(24, 37.5,Point.CARTESIAN))); //new Point(21, 0, Point.CARTESIAN),
        toBackboardMiddle.setConstantHeadingInterpolation(Math.toRadians(90));

        toBackboardLeft = new Path(new BezierLine((new Point(21.8, 24, Point.CARTESIAN)),(new Point(19.5, 38,Point.CARTESIAN))));
        toBackboardLeft.setConstantHeadingInterpolation(Math.toRadians(90));

        toBackboardRight = new Path(new BezierLine(new Point(22.3,3,Point.CARTESIAN),new Point(31,38,Point.CARTESIAN)));
        toBackboardRight.setConstantHeadingInterpolation(Math.toRadians(90));

        toStackLeft = new Path(new BezierCurve(new Point(20, 36.5, Point.CARTESIAN), new Point(-2, 7,Point.CARTESIAN),new Point(-2, -15,Point.CARTESIAN),new Point(-2, -33,Point.CARTESIAN),(new Point(-3, -60, Point.CARTESIAN)),(new Point(18,-67.5,Point.CARTESIAN))));
        toStackLeft.setConstantHeadingInterpolation(Math.toRadians(90));
        toStackLeft2 = new Path(new BezierCurve(new Point(20, 36.5, Point.CARTESIAN), new Point(-2, 7,Point.CARTESIAN),new Point(-2, -15,Point.CARTESIAN),new Point(-2, -33,Point.CARTESIAN),(new Point(-2, -60, Point.CARTESIAN)),(new Point(18,-67.5,Point.CARTESIAN))));
        toStackLeft2.setConstantHeadingInterpolation(Math.toRadians(90));
//        toStackLeft.setReversed(true);
        toStrafeAtStackLeft = new Path(new BezierLine((new Point(18,-68,Point.CARTESIAN)),(new Point(24.5 ,-69,Point.CARTESIAN))));
        toStrafeAtStackLeft.setConstantHeadingInterpolation(Math.toRadians(90));
        toStrafeAtStackLeft2 = new Path(new BezierLine((new Point(18,-68,Point.CARTESIAN)),(new Point(24.5,-69,Point.CARTESIAN))));
        toStrafeAtStackLeft2.setConstantHeadingInterpolation(Math.toRadians(90));

//        toStackMiddle = new Path(new BezierCurve(new Point(23, 36, Point.CARTESIAN), new Point(56.6, 33,Point.CARTESIAN),new Point(65, 0,Point.CARTESIAN),(new Point(34, -68, Point.CARTESIAN))));
//        toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));
//        toStackMiddle.setReversed(true);
//        toStrafeAtStackMiddle = new Path(new BezierLine((new Point(34,-67.5,Point.CARTESIAN)),(new Point(30 ,-67.5,Point.CARTESIAN))));
//        toStrafeAtStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));

        toBackboardFromStack = new Path(new BezierCurve((new Point(19.8,-71.3,Point.CARTESIAN)),(new Point(-2.6, -59, Point.CARTESIAN)),(new Point(-2.6, -33,Point.CARTESIAN)),new Point(-1.5, -15,Point.CARTESIAN),(new Point(-1.5, 7,Point.CARTESIAN)),(new Point(2, 23,Point.CARTESIAN)),new Point(20, 36.5, Point.CARTESIAN)));
        toBackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(90));

        toPark = new Path(new BezierLine(new Point(24,37.5,Point.CARTESIAN),new Point(1,36,Point.CARTESIAN)));

        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.BLUE;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);


//        robot.follower.setStartingPose(new Pose(-60,34,Math.toRadians(90)));



        while (opModeInInit()) {
            CommandScheduler.getInstance().reset();

            zone= robot.propDetectionPipeline.detectZone();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new followPath(CenterstageConstants.getPath(zone, toSpikeLeft,toSpikeMiddle,toSpikeRight)),
                            new WaitCommand(300),
                            new pitchToSpikePosition(),
                            new WaitCommand(100),
                            new pivotToSpikePosition(),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new releaseLeftPixel(),
                            new WaitCommand(100),
                            new followPath(CenterstageConstants.getPath(zone, toBackboardLeft,toBackboardMiddle,toBackboardRight)),
                            new pitchToDropPosition(),
                            new pivotToDropPosition(),

                            new WaitUntilCommand(busy),
                            new WaitCommand(100),
                            new releaseRightPixel(),
                            new WaitCommand(150),
                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),

                            new followPath(toStackLeft),
                            new v4BarToHeight(5),
                            new WaitUntilCommand(busy),
                            new outtakeCommand(),
                            new WaitCommand(700),
                            new outtakeCommand(),
                            new followPath(toStrafeAtStackLeft),
                            new WaitCommand(150),
                            new intakeCommand(),
                            new WaitCommand(80),
                            new outtakeCommand(),
                            new v4BarToHeight(4),
                            new WaitCommand(400),
                            new v4BarToHeight(3),
                            new WaitCommand(300),

                            new WaitUntilCommand(busy),
                            new ParallelRaceGroup(
                                    new WaitUntilCommand(pixels),
                                    new WaitCommand(900)
                            ),
                            new stopIntake(),
                            new WaitCommand(400),
                            new followPath(toBackboardFromStack),
                            new pivotToTransferPosition(),
                            new WaitCommand(200),
                            new pitchToTransferPosition(),
                            new WaitCommand(400),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new intakeCommand(),
                            new WaitCommand(400),
                            new stopIntake(),
                            new WaitUntilCommand(pastCenter),
                            new pitchToDropPosition(),
                            new WaitCommand(80),
                            new pivotToDropPosition(),
                            new slideToRow(3),
                            new WaitUntilCommand(busy),
                            new WaitCommand(100),
                            new releaseRightPixel(),
                            new releaseLeftPixel(),
                            new WaitCommand(150),
//
                            new slideToRow(1),
                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),

                            new followPath(toStackLeft2),
                            new v4BarToHeight(3),
                            new WaitUntilCommand(busy),
                            new outtakeCommand(),
                            new WaitCommand(700),
                            new v4BarToHeight(2),
                            new followPath(toStrafeAtStackLeft2),
                            new WaitCommand(150),
                            new intakeCommand(),
                            new WaitCommand(80),
                            new outtakeCommand(),
                            new WaitCommand(100),
                            new v4BarToHeight(1),
//
//                            new v4BarToHeight(3),
//                            new WaitCommand(150),
//                            new v4BarToHeight(2),
//                            new WaitCommand(150),
//                            new v4BarToHeight(1),
//                            new WaitCommand(150),

                            new WaitUntilCommand(busy),
                            new ParallelRaceGroup(
                                    new WaitUntilCommand(pixels),
                                    new WaitCommand(900)
                            ),
                            new stopIntake(),
                            new WaitCommand(200),
                            new pivotToTransferPosition(),
                            new WaitCommand(100),
                            new pitchToTransferPosition(),

                            new followPath(toBackboardFromStack),
                            new WaitCommand(400),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new intakeCommand(),
                            new WaitCommand(200),
                            new stopIntake(),
                            new WaitUntilCommand(pastCenter),
                            new pitchToDropPosition(),
                            new WaitCommand(80),
                            new pivotToDropPosition(),
                            new slideToRow(3),
                            new WaitUntilCommand(busy),
                            new WaitCommand(100),
                            new releaseRightPixel(),
                            new releaseLeftPixel(),
                            new WaitCommand(150),


                            new slideToRow(1),
                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),

                            new followPath(toPark),
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
        robot.stopCameraStream();

        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();



    }

}
