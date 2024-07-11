package org.firstinspires.ftc.teamcode.opmode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.onbotjava.handlers.objbuild.WaitForBuild;
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
import org.opencv.core.Mat;

import java.util.function.BooleanSupplier;


@Autonomous
public class AutoBlueCloseNewBezier extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toStrafeAtStackLeft;
    private Path fromBBtoStackfullcurve;
    private Path tobackboardFromStack;
    private Path tocurvetostackBack;
    public boolean closed = false;
    private Path toPark;
    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >26;
    private BooleanSupplier pastCenter = () -> robot.follower.getPose().getX()>30;
    private BooleanSupplier closeToStack = () -> robot.follower.getPose().getX()<-40;
    private BooleanSupplier backtoBBcurvedone = () -> robot.follower.getPose().getX()>7;
    private BooleanSupplier ultrasonic = () -> robot.doubleSubscriber(Sensors.SensorType.LEFT_DISTANCE)>60;
    private int zone;
    private Location randomization;

    @Override
    public void initialize() {
        telemetry.setMsTransmissionInterval(50);



        toPark = new Path(new BezierLine(new Point(46, 39.5, Point.CARTESIAN), new Point(46, 55, Point.CARTESIAN)));
        toPark.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

        toSpikeMiddle = new Path(new BezierLine(new Point(9, 54, Point.CARTESIAN), new Point(26, 23, Point.CARTESIAN)));
        toSpikeMiddle.setConstantHeadingInterpolation(Math.toRadians(180)); // -180 * -1 = 180 degrees

        toSpikeLeft = new Path(new BezierCurve(new Point(9, 54, Point.CARTESIAN), new Point(22, 45, Point.CARTESIAN), new Point(11.7, 34.5, Point.CARTESIAN)));
        toSpikeLeft.setConstantHeadingInterpolation(Math.toRadians(180)); // -180 * -1 = 180 degrees

        toSpikeRight = new Path(new BezierLine(new Point(9, 54, Point.CARTESIAN), new Point(31, 34.5, Point.CARTESIAN)));
        toSpikeRight.setConstantHeadingInterpolation(Math.toRadians(180)); // -180 * -1 = 180 degrees

        toBackboardMiddle = new Path(new BezierLine(new Point(26, 25, Point.CARTESIAN), new Point(42, 35, Point.CARTESIAN)));
        toBackboardMiddle.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

        toBackboardLeft = new Path(new BezierLine(new Point(11.7, 35.5, Point.CARTESIAN), new Point(42, 27, Point.CARTESIAN)));
        toBackboardLeft.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

        toBackboardRight = new Path(new BezierLine(new Point(31, 35, Point.CARTESIAN), new Point(42, 39, Point.CARTESIAN)));
        toBackboardRight.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

        fromBBtoStackfullcurve = new Path(new BezierCurve(
                new Point(46, 35, Point.CARTESIAN),
                new Point(34, 35, Point.CARTESIAN),
                new Point(34, 60, Point.CARTESIAN),
                new Point(10, 60, Point.CARTESIAN),
                new Point(-26, 60, Point.CARTESIAN),
                new Point(-46, 60, Point.CARTESIAN),
                new Point(-50, 60, Point.CARTESIAN),
                new Point(-50, 44, Point.CARTESIAN),
                new Point(-57, 44, Point.CARTESIAN)));
        fromBBtoStackfullcurve.setConstantHeadingInterpolation(Math.toRadians(-3)); // 3 * -1 = -3 degrees

        toStrafeAtStackLeft = new Path(new BezierLine(new Point(-59, 44, Point.CARTESIAN), new Point(-60, 35, Point.CARTESIAN)));
        toStrafeAtStackLeft.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

        tocurvetostackBack = new Path(new BezierCurve(new Point(-60, 35, Point.CARTESIAN), new Point(-50, 35, Point.CARTESIAN), new Point(-50, 58, Point.CARTESIAN), new Point(-46, 58, Point.CARTESIAN), new Point(-26, 58, Point.CARTESIAN), new Point(10, 58, Point.CARTESIAN)));
        tocurvetostackBack.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

        tobackboardFromStack = new Path(new BezierLine(new Point(10, 59, Point.CARTESIAN), new Point(44.5, 35, Point.CARTESIAN)));
        tobackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(5)); // -5 * -1 = 5 degrees




        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.RED;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);


        robot.follower.setStartingPose(new Pose(9,-58,Math.toRadians(90)));
        while (opModeInInit()) {

            zone= robot.propDetectionPipeline.detectZone();
            CommandScheduler.getInstance().reset();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new followPath(CenterstageConstants.getPath(zone,toSpikeLeft,toSpikeMiddle,toSpikeRight)),
                                    new SequentialCommandGroup(
                                            new pitchToTransferPosition(),
                                            new pivotToTransferPosition(),
                                            new WaitCommand(200),
                                            new grabRightPixel(),
                                            new grabLeftPixel(),
                                            new WaitCommand(700),
                                            new pitchToSpikePosition(),
                                            new WaitCommand(100),
                                            new pivotToSpikePosition(),
                                            new WaitCommand(300)
                                    ),
                                    new WaitUntilCommand(busy)
                            ),
                            new WaitCommand(200),
                            new releaseRightPixel(),
                            new WaitCommand(200),
                            new stopIntake(),
                            new followPath(CenterstageConstants.getPath(zone,toBackboardLeft,toBackboardMiddle,toBackboardRight)),
                            new WaitUntilCommand(pastCenter),
                            new pitchToDropPosition(),
                            new pivotToDropPosition(),
                            new WaitUntilCommand(busy),
                            new WaitCommand(200),
                            new releaseLeftPixel(),
                            new WaitCommand(120),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new intakeToHang(),
                            new followPath(fromBBtoStackfullcurve),
                            new WaitUntilCommand(closeToStack),
                            new outtakeCommand(),
                            new v4BarToHeight(5),
                            new WaitUntilCommand(busy),
                            new WaitCommand(200),
                            new followPath(toStrafeAtStackLeft),
                            new v4BarToHeight(4),
                            new WaitCommand(400),
                            new v4BarToHeight(3),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new followPath(tocurvetostackBack),
                            new intakeCommand(),
                            new WaitCommand(150),
                            new outtakeCommand(),
                            new WaitCommand(500),
                            new intakeCommand(),
                            new WaitCommand(300),
                            new stopIntake(),
                            new WaitCommand(400),
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(400),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new WaitUntilCommand(backtoBBcurvedone),
                            new WaitUntilCommand(ultrasonic),
                            new followPath(tobackboardFromStack),
                            new WaitUntilCommand(pastCenter),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new releaseLeftPixel(),
                            new releaseRightPixel(),
                            new WaitCommand(200),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new WaitCommand(200),
                            new intakeToHang(),
                            new followPath(fromBBtoStackfullcurve),
                            new WaitUntilCommand(closeToStack),
                            new outtakeCommand(),
                            new v4BarToHeight(3),
                            new WaitUntilCommand(busy),
                            new WaitCommand(300),
                            new followPath(toStrafeAtStackLeft),
                            new v4BarToHeight(2),
                            new WaitCommand(300),
                            new v4BarToHeight(1),
                            new WaitCommand(200),
                            new WaitUntilCommand(busy),
                            new followPath(tocurvetostackBack),
                            new intakeCommand(),
                            new WaitCommand(200),
                            new outtakeCommand(),
                            new WaitCommand(300),
                            new intakeCommand(),
                            new WaitCommand(150),
                            new stopIntake(),
                            new WaitCommand(400),
                            new pivotToTransferPosition(),
                            new pitchToTransferPosition(),
                            new WaitCommand(400),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new WaitUntilCommand(backtoBBcurvedone),
                            new WaitUntilCommand(ultrasonic),
                            new followPath(tobackboardFromStack),
                            new WaitUntilCommand(pastCenter),
                            new slideToRow(3),
                            new pitchToDropPosition(),
                            new WaitCommand(100),
                            new pivotToDropPosition(),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new releaseLeftPixel(),
                            new releaseRightPixel(),
                            new WaitCommand(200),
                            new slideToRow(1),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new followPath(toPark),
                            new intakeToHang()
                    )
            );
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
        telemetry.addData("joemamaobama",robot.getCameraState());

    }
}