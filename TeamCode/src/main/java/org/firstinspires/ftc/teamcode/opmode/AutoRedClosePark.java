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
import org.opencv.core.Mat;

import java.util.function.BooleanSupplier;


@Autonomous
public class AutoRedClosePark extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toBackboard;
    public boolean closed = false;
    private Path toPark;
    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >26;
    private BooleanSupplier pastCenter = () -> robot.follower.getPose().getX()>24;
    private BooleanSupplier closeToStack = () -> robot.follower.getPose().getX()<-40;
    private BooleanSupplier backtoBBcurvedone = () -> robot.follower.getPose().getX()>7;
    private BooleanSupplier ultrasonic = () -> robot.doubleSubscriber(Sensors.SensorType.LEFT_DISTANCE)>60;
    private int zone;
    private Location randomization;
    double spikeDropX;
    double spikeDropY;
    double bbDropY;
    double bbDropX;
    double whiteDropX=45;
    double whiteDropY=-33;

    @Override
    public void initialize() {
        telemetry.setMsTransmissionInterval(50);


        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        CenterstageConstants.ALLIANCE = Location.RED;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);


        robot.follower.setStartingPose(new Pose(9,-58,Math.toRadians(90)));
        while (opModeInInit()) {

            zone= robot.propDetectionPipeline.detectZone();
            CommandScheduler.getInstance().reset();

            if (zone==1){
                spikeDropX=11.7;
                spikeDropY=-34.5;
                bbDropY=-27;
                bbDropX=45;
            }
            else if (zone==2) {
                spikeDropX=26;
                spikeDropY=-23;
                bbDropY=-33;
                bbDropX=45;
            }
            else{
                spikeDropX=31;
                spikeDropY=-34.5;
                bbDropY=-39;
                bbDropX=45;
            }

            toPark = new Path(new BezierLine(new Point(42.5, -35, Point.CARTESIAN), new Point(42.5, -5, Point.CARTESIAN)));
            toPark.setConstantHeadingInterpolation(Math.toRadians(0)); // -90 + 90 = 0 degrees

            toSpikeMiddle = new Path(new BezierLine(new Point(9, -54, Point.CARTESIAN), new Point(spikeDropX, spikeDropY, Point.CARTESIAN)));
            toSpikeMiddle.setConstantHeadingInterpolation(Math.toRadians(-180)); // 90 + 90 = 180 degrees

            toSpikeLeft = new Path(new BezierCurve(new Point(9, -54, Point.CARTESIAN), new Point(22, -45, Point.CARTESIAN), new Point(spikeDropX, spikeDropY, Point.CARTESIAN)));
            toSpikeLeft.setConstantHeadingInterpolation(Math.toRadians(-180)); // 90 + 90 = 180 degrees

            toSpikeRight = new Path(new BezierLine(new Point(9, -54, Point.CARTESIAN), new Point(spikeDropX, spikeDropY, Point.CARTESIAN)));
            toSpikeRight.setConstantHeadingInterpolation(Math.toRadians(-180)); // 90 + 90 = 180 degrees


            toBackboard = new Path(new BezierLine(
                    new Point(spikeDropX, spikeDropY, Point.CARTESIAN),
                    new Point(bbDropX, bbDropY, Point.CARTESIAN)));
            toBackboard.setConstantHeadingInterpolation(Math.toRadians(0)); // -90 + 90 = 0 degrees

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
                            new followPath(toBackboard), //CenterstageConstants.getPath(zone,toBackboardLeft,toBackboardMiddle,toBackboardRight)
                            new WaitUntilCommand(pastCenter),
                            new pitchToDropPosition(),
                            new pivotToDropPosition(),
                            new WaitUntilCommand(busy),
                            new WaitCommand(200),
                            new releaseLeftPixel(),
                            new WaitCommand(120),

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