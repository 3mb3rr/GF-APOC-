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
public class AutoBlueCloseCenter extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Path toSpikeLeft;
    private Path toSpikeMiddle;
    private Path toSpikeRight;
    private Path toBackboard;
    //    private Path toBackboardRight;
//    private Path toBackboardMiddle;
//    private Path toBackboardLeft;
    private Path toStackMFromBB;
    private Path toStrafeAtMStack;
    private Path toBackboardfromstack;
    private Path toBackboardSecondTime;
    private Path TostackSecondTime;
    private Path TostrafeSecondTime;

    private Path toStrafeAtStackLeft;
    private Path fromBBtoStackfullcurve;
    private Path tobackboardFromStack;
    private Path tocurvetostackBack;
    public boolean closed = false;
    private Path toPark;
    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >26;
    private BooleanSupplier pastCenter = () -> robot.follower.getPose().getX()>25;
    private BooleanSupplier closeToStack = () -> robot.follower.getPose().getX()<-40;
    private BooleanSupplier backtoBBcurvedone = () -> robot.follower.getPose().getX()>7;
    private BooleanSupplier ultrasonic = () -> robot.doubleSubscriber(Sensors.SensorType.LEFT_DISTANCE)>60;
    private int zone;
    double spikeDropX;
    double spikeDropY;
    double bbDropY;
    double bbDropX;
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


        robot.follower.setStartingPose(new Pose(9,58,Math.toRadians(-90)));
        while (opModeInInit()) {

            zone= robot.propDetectionPipeline.detectZone();
            CommandScheduler.getInstance().reset();

            if (zone==1){
                spikeDropX=32;
                spikeDropY=29.5;
                bbDropY=27;
                bbDropX=44;
            }
            else if (zone==2) {
                spikeDropX=26;
                spikeDropY=23;
                bbDropY=33;
                bbDropX=44;
            }
            else{
                spikeDropX=12;
                spikeDropY=36;
                bbDropY=39;
                bbDropX=44;
            }

            toPark = new Path(new BezierLine(new Point(43.5, 35, Point.CARTESIAN), new Point(43.5, 55, Point.CARTESIAN)));
            toPark.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

            toSpikeMiddle = new Path(new BezierLine(new Point(9, 54, Point.CARTESIAN), new Point(spikeDropX, spikeDropY, Point.CARTESIAN)));
            toSpikeMiddle.setConstantHeadingInterpolation(Math.toRadians(180)); // -180 * -1 = 180 degrees

            toSpikeLeft = new Path(new BezierCurve(new Point(9, 58, Point.CARTESIAN), new Point(22, 45, Point.CARTESIAN), new Point(spikeDropX, spikeDropY, Point.CARTESIAN)));
            toSpikeLeft.setLinearHeadingInterpolation(0, Math.toRadians(180));

            toSpikeRight = new Path(new BezierCurve(new Point(9, 58, Point.CARTESIAN), new Point(15, 42, Point.CARTESIAN), new Point(spikeDropX, spikeDropY, Point.CARTESIAN)));
            toSpikeRight.setLinearHeadingInterpolation(0, Math.toRadians(180));

            toBackboard = new Path(new BezierCurve(new Point(spikeDropX,spikeDropY,Point.CARTESIAN),new Point(bbDropX,bbDropY,Point.CARTESIAN)));

//            toBackboardMiddle = new Path(new BezierLine(new Point(26, 25, Point.CARTESIAN), new Point(43.5, 35, Point.CARTESIAN)));
//            toBackboardMiddle.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees
//
//            toBackboardLeft = new Path(new BezierLine(new Point(11.7, 35.5, Point.CARTESIAN), new Point(43.5, 27, Point.CARTESIAN)));
//            toBackboardLeft.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees
//
//            toBackboardRight = new Path(new BezierLine(new Point(31, 35, Point.CARTESIAN), new Point(43.5, 39, Point.CARTESIAN)));
//            toBackboardRight.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

            toStackMFromBB = new Path(new BezierCurve(
                    (new Point(bbDropX,bbDropY,Point.CARTESIAN)),
                    (new Point(30,4,Point.CARTESIAN)),
                    (new Point(-27,4,Point.CARTESIAN)),
                    (new Point(-57, 15,Point.CARTESIAN))));
            toStackMFromBB.setConstantHeadingInterpolation(0);

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


            fromBBtoStackfullcurve = new Path(new BezierCurve(
                    new Point(43.5, bbDropY, Point.CARTESIAN),
                    new Point(34, bbDropY, Point.CARTESIAN),
                    new Point(34, 60, Point.CARTESIAN),
                    new Point(22,60,Point.CARTESIAN),
                    new Point(10, 60, Point.CARTESIAN),
                    new Point(-26, 60, Point.CARTESIAN),
                    new Point(-46, 60, Point.CARTESIAN),
                    new Point(-50, 60, Point.CARTESIAN),
                    new Point(-50, 44, Point.CARTESIAN),
                    new Point(-57, 44, Point.CARTESIAN)));
            fromBBtoStackfullcurve.setConstantHeadingInterpolation(Math.toRadians(-3)); // 3 * -1 = -3 degrees

            toStrafeAtStackLeft = new Path(new BezierLine(new Point(-57, 44, Point.CARTESIAN), new Point(-59, 35, Point.CARTESIAN)));
            toStrafeAtStackLeft.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

            tocurvetostackBack = new Path(new BezierCurve(
                    new Point(-59, 35, Point.CARTESIAN),
                    new Point(-51, 35, Point.CARTESIAN),
                    new Point(-51, 61.5, Point.CARTESIAN),
                    new Point(-46, 59, Point.CARTESIAN),
                    new Point(-26, 59, Point.CARTESIAN),
                    new Point(-5, 59, Point.CARTESIAN),
                    new Point(10, 57, Point.CARTESIAN)));
            tocurvetostackBack.setConstantHeadingInterpolation(Math.toRadians(0)); // 0 * -1 = 0 degrees

            tobackboardFromStack = new Path(new BezierLine(new Point(10, 58, Point.CARTESIAN), new Point(43.5, 35, Point.CARTESIAN)));
            tobackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(5)); // -5 * -1 = 5 degrees

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
                            new releaseLeftPixel(),
                            new WaitCommand(200),
                            new stopIntake(),
                            new followPath(toBackboard),
                            new WaitUntilCommand(pastCenter),
                            new pitchToDropPosition(),
                            new pivotToDropPosition(),
                            new WaitUntilCommand(busy),
                            new WaitCommand(200),
                            new releaseRightPixel(),
                            new WaitCommand(120),
                            new pitchToWaitPosition(),
                            new pivotToWaitPosition(),
                            new intakeToHang(),
                            new followPath(toStackMFromBB),
                            new WaitUntilCommand(closeToStack),
                            new outtakeCommand(),
                            new v4BarToHeight(5),
                            new WaitUntilCommand(busy),
                            new WaitCommand(200),
                            new followPath(toStrafeAtMStack),
                            new v4BarToHeight(4),
                            new WaitCommand(400),
                            new v4BarToHeight(3),
                            new WaitCommand(300),
                            new WaitUntilCommand(busy),
                            new followPath(toBackboardfromstack),
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
                            new followPath(TostackSecondTime),
                            new WaitUntilCommand(closeToStack),
                            new outtakeCommand(),
                            new v4BarToHeight(3),
                            new WaitUntilCommand(busy),
                            new WaitCommand(300),
                            new followPath(TostrafeSecondTime),
                            new v4BarToHeight(2),
                            new WaitCommand(300),
                            new v4BarToHeight(1),
                            new WaitCommand(200),
                            new WaitUntilCommand(busy),
                            new followPath(toBackboardSecondTime),
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