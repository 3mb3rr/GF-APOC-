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
    private Path toStackMiddle;
    private Path toStrafeAtStackLeft;
    private Path toStrafeAtStackMiddle;
    private Path toBackboardRight;
    private Path toBackboardMiddle;
    private Path toBackboardLeft;
    private Path toBackboardFromStack;
    private Path toPark;
    private BooleanSupplier busy = () -> !robot.follower.isBusy();
    private BooleanSupplier pixels = () -> robot.intake.getLeftPixel() && robot.intake.getRightPixel();
    private BooleanSupplier time = () -> robot.getTimeSec() >26;

    private int zone;
    @Override
    public void initialize() {


        telemetry.setMsTransmissionInterval(50);

        toSpikeMiddle = new Path(new BezierLine(new Point(0, 0,Point.CARTESIAN),new Point(33.5, 17,Point.CARTESIAN)));
        toSpikeMiddle.setLinearHeadingInterpolation(0,Math.toRadians(-90));

        toSpikeLeft = new Path(new BezierCurve(new Point(0, 0,Point.CARTESIAN), new Point(13,13,Point.CARTESIAN), new Point(28.5,24,Point.CARTESIAN)));
        toSpikeLeft.setLinearHeadingInterpolation(0,Math.toRadians(-90));

        toSpikeRight = new Path(new BezierCurve(new Point(0, 0,Point.CARTESIAN), new Point(16,6,Point.CARTESIAN),new Point(22, 3,Point.CARTESIAN)));
        toSpikeRight.setLinearHeadingInterpolation(0,Math.toRadians(-90));

        toBackboardMiddle = new Path(new BezierLine(new Point(21, 0, Point.CARTESIAN),new Point(24, 37.5,Point.CARTESIAN))); //new Point(21, 0, Point.CARTESIAN),
        toBackboardMiddle.setConstantHeadingInterpolation(Math.toRadians(90));

        toBackboardLeft = new Path(new BezierLine((new Point(21.8, 24, Point.CARTESIAN)),(new Point(21, 38,Point.CARTESIAN))));
        toBackboardLeft.setConstantHeadingInterpolation(Math.toRadians(90));

        toBackboardRight = new Path(new BezierLine(new Point(22.3,3,Point.CARTESIAN),new Point(31,38,Point.CARTESIAN)));
        toBackboardRight.setConstantHeadingInterpolation(Math.toRadians(90));

        toStackLeft = new Path(new BezierCurve(new Point(20, 36.5, Point.CARTESIAN), new Point(-2, 11,Point.CARTESIAN),new Point(-2, -21,Point.CARTESIAN),(new Point(-2, -58, Point.CARTESIAN)),(new Point(22,-67,Point.CARTESIAN))));
        toStackLeft.setConstantHeadingInterpolation(Math.toRadians(90));
//        toStackLeft.setReversed(true);
        toStrafeAtStackLeft = new Path(new BezierLine((new Point(22,-68,Point.CARTESIAN)),(new Point(24.5,-69,Point.CARTESIAN))));
        toStrafeAtStackLeft.setConstantHeadingInterpolation(Math.toRadians(90));

//        toStackMiddle = new Path(new BezierCurve(new Point(23, 36, Point.CARTESIAN), new Point(56.6, 33,Point.CARTESIAN),new Point(65, 0,Point.CARTESIAN),(new Point(34, -68, Point.CARTESIAN))));
//        toStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));
//        toStackMiddle.setReversed(true);
//        toStrafeAtStackMiddle = new Path(new BezierLine((new Point(34,-67.5,Point.CARTESIAN)),(new Point(30 ,-67.5,Point.CARTESIAN))));
//        toStrafeAtStackMiddle.setConstantHeadingInterpolation(Math.toRadians(-85));

        toBackboardFromStack = new Path(new BezierCurve((new Point(19.8,-71.3,Point.CARTESIAN)),(new Point(-2.4, -58, Point.CARTESIAN)),(new Point(-2.4, -21,Point.CARTESIAN)),(new Point(-2.4, 11,Point.CARTESIAN)),new Point(20, 37, Point.CARTESIAN)));
        toBackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(90));

        toPark = new Path(new BezierLine(new Point(24,37.5,Point.CARTESIAN),new Point(5,36,Point.CARTESIAN)));

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
                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),

                            new followPath(toStackLeft),
                            new v4BarToHeight(5),
                            new WaitUntilCommand(busy),
                            new outtakeCommand(),
                            new WaitCommand(350),
                            new followPath(toStrafeAtStackLeft),
                            new v4BarToHeight(4),
                            new WaitCommand(150),
                            new v4BarToHeight(3),
                            new WaitCommand(150),

//                            new v4BarToHeight(5),
//                            new WaitCommand(150),
//                            new v4BarToHeight(4),
//                            new WaitCommand(150),
//                            new v4BarToHeight(3),
//                            new WaitCommand(150),

                            new WaitUntilCommand(busy),
                            new WaitCommand(150),
                            new v4BarToHeight(4),
                            new ParallelRaceGroup(
                                    new WaitUntilCommand(pixels),
                                    new WaitCommand(1500)
                            ),
                            new stopIntake(),
                            new WaitCommand(400),
                            new followPath(toBackboardFromStack),
                            new pivotToTransferPosition(),
                            new WaitCommand(200),
                            new pitchToTransferPosition(),
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
                            new WaitCommand(200),
                            new releaseRightPixel(),
                            new releaseLeftPixel(),
//
                            new slideToRow(1),
                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),

                            new followPath(toStackLeft),
                            new v4BarToHeight(3),
                            new WaitUntilCommand(busy),
                            new outtakeCommand(),
                            new WaitCommand(750),
                            new followPath(toStrafeAtStackLeft),
                            new v4BarToHeight(2),
                            new WaitCommand(100),
                            new v4BarToHeight(1),
                            new WaitCommand(100),
//
//                            new v4BarToHeight(3),
//                            new WaitCommand(150),
//                            new v4BarToHeight(2),
//                            new WaitCommand(150),
//                            new v4BarToHeight(1),
//                            new WaitCommand(150),

                            new WaitUntilCommand(busy),
                            new WaitCommand(50),
                            new ParallelRaceGroup(
                                    new WaitUntilCommand(pixels),
                                    new WaitCommand(1000)
                            ),
                            new stopIntake(),
                            new WaitCommand(200),
                            new pivotToTransferPosition(),
                            new WaitCommand(100),
                            new pitchToTransferPosition(),

                            new followPath(toBackboardFromStack),
                            new WaitCommand(500),
                            new grabLeftPixel(),
                            new grabRightPixel(),
                            new intakeCommand(),
                            new WaitCommand(200),
                            new stopIntake(),
                            new WaitUntilCommand(busy),
                            new pitchToDropPosition(),
                            new WaitCommand(80),
                            new pivotToDropPosition(),
                            new slideToRow(3),
                            new WaitCommand(400),
                            new releaseRightPixel(),
                            new releaseLeftPixel(),
                            new WaitCommand(100),


                            new slideToRow(1),
                            new pivotToWaitPosition(),
                            new pitchToWaitPosition(),

                            new followPath(toPark)


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
