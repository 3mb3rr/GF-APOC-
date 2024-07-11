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

import java.util.function.BooleanSupplier;


@Autonomous
public class AutoBlueCloseGlobal extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toStackLeft;
    private Path toStackLeft2;
    private Path toStrafeAtStackLeft;
    private Path toStrafeAtStackLeft2;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toBackboardFromStack;
    private Path toBackboardFromMid;
    private Path toPark;
    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >26;
    private BooleanSupplier pastCenter = () -> robot.follower.getPose().getX()>30;
    private BooleanSupplier closeToStack = () -> robot.follower.getPose().getX()<-40;
    private BooleanSupplier backtoBBcurvedone = () -> robot.follower.getPose().getX()>17;
    private BooleanSupplier ultrasonic = () -> robot.doubleSubscriber(Sensors.SensorType.RIGHT_DISTANCE)>60;

    private int zone;
    @Override
    public void initialize() {


        telemetry.setMsTransmissionInterval(50);

        toSpikeMiddle = new Path(new BezierLine(new Point(9, 58, Point.CARTESIAN), new Point(26.5, 24.5, Point.CARTESIAN)));
        toSpikeMiddle.setLinearHeadingInterpolation(0, Math.toRadians(-180));

        toSpikeLeft = new Path(new BezierCurve(new Point(9, 58, Point.CARTESIAN), new Point(22, 45, Point.CARTESIAN), new Point(32, 29.5, Point.CARTESIAN)));
        toSpikeLeft.setLinearHeadingInterpolation(0, Math.toRadians(-180));

        toSpikeRight = new Path(new BezierCurve(new Point(9, 58, Point.CARTESIAN), new Point(15, 42, Point.CARTESIAN), new Point(12, 36, Point.CARTESIAN)));
        toSpikeRight.setLinearHeadingInterpolation(0, Math.toRadians(-180));

        toBackboardMiddle = new Path(new BezierLine(new Point(9, 37, Point.CARTESIAN), new Point(46.5, 34, Point.CARTESIAN)));
        toBackboardMiddle.setConstantHeadingInterpolation(Math.toRadians(0));

        toBackboardLeft = new Path(new BezierLine(new Point(33, 36.2, Point.CARTESIAN), new Point(46.5, 38.5, Point.CARTESIAN)));
        toBackboardLeft.setConstantHeadingInterpolation(Math.toRadians(0));

        toBackboardRight = new Path(new BezierLine(new Point(12, 36, Point.CARTESIAN), new Point(46.5, 27, Point.CARTESIAN)));
        toBackboardRight.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackLeft = new Path(new BezierCurve(new Point(45.5, 38, Point.CARTESIAN), new Point(16, 60, Point.CARTESIAN), new Point(-6, 60, Point.CARTESIAN), new Point(-24, 60, Point.CARTESIAN), new Point(-51, 61, Point.CARTESIAN), new Point(-57.5, 40, Point.CARTESIAN)));
        toStackLeft.setConstantHeadingInterpolation(Math.toRadians(0));

        toStackLeft2 = new Path(new BezierCurve(new Point(45.5, 38, Point.CARTESIAN), new Point(16, 60, Point.CARTESIAN), new Point(-6, 60, Point.CARTESIAN), new Point(-24, 60, Point.CARTESIAN), new Point(-51, 60, Point.CARTESIAN), new Point(-57.5, 40, Point.CARTESIAN)));
        toStackLeft2.setConstantHeadingInterpolation(Math.toRadians(0));

        toStrafeAtStackLeft = new Path(new BezierLine(new Point(-58.5, 40, Point.CARTESIAN), new Point(-59.5, 33.5, Point.CARTESIAN)));
        toStrafeAtStackLeft.setConstantHeadingInterpolation(Math.toRadians(0));

        toStrafeAtStackLeft2 = new Path(new BezierLine(new Point(-58.5, 40, Point.CARTESIAN), new Point(-59.5, 33.5, Point.CARTESIAN)));
        toStrafeAtStackLeft2.setConstantHeadingInterpolation(Math.toRadians(0));

        toBackboardFromStack = new Path(new BezierCurve(new Point(-62.3, 38.2, Point.CARTESIAN), new Point(-50, 60.6, Point.CARTESIAN), new Point(-24, 60.6, Point.CARTESIAN), new Point(-6, 59.5, Point.CARTESIAN), new Point(19, 59.5, Point.CARTESIAN)));
        toBackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(0));

        toBackboardFromMid = new Path(new BezierLine(new Point(19, 59.5, Point.CARTESIAN), new Point(45.5, 38, Point.CARTESIAN)));
        toBackboardFromMid.setConstantHeadingInterpolation(Math.toRadians(0));

        toPark = new Path(new BezierLine(new Point(46.5, 34, Point.CARTESIAN), new Point(45, 57, Point.CARTESIAN)));

        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.BLUE;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        robot.follower.setStartingPose(new Pose(9,58,Math.toRadians(-90)));


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
                            new WaitCommand(300),
                            new stopIntake(),
                            new WaitUntilCommand(backtoBBcurvedone),
                            new WaitUntilCommand(ultrasonic),
                            new followPath(toBackboardFromMid),
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
                            new WaitUntilCommand(backtoBBcurvedone),
                            new WaitUntilCommand(ultrasonic),
                            new followPath(toBackboardFromMid),
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
