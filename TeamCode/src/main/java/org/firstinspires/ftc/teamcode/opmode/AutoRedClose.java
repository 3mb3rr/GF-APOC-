package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import java.util.*;
import java.util.function.BooleanSupplier;


@TeleOp
public class AutoRedClose extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Path toSpike;
    private Path toStack;
    private Path toBackboard;
    private Path toBackboardFromStack;
    private Path toPark;
    private BooleanSupplier busy = () -> !robo  t.follower.isBusy();


    @Override
    public void initialize() {
//        robot.follower.setStartingPose();
        toSpike = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(22, 0,Point.CARTESIAN)));
        toSpike.setConstantHeadingInterpolation(Math.toRadians(0));
        toBackboard = new Path(new BezierLine(new Point(22, 0, Point.CARTESIAN), new Point(23, -38,Point.CARTESIAN)));
        toBackboard.setConstantHeadingInterpolation(Math.toRadians(-90));
        toStack = new Path(new BezierCurve(new Point(23, -38, Point.CARTESIAN), new Point(56.6, -33,Point.CARTESIAN),new Point(71.3, 0,Point.CARTESIAN),(new Point(46.3, 67.4, Point.CARTESIAN))));
        toStack.setConstantHeadingInterpolation(Math.toRadians(-90));
        toStack.setReversed(true);
        toBackboardFromStack = new Path(new BezierCurve((new Point(46.3, 67.4, Point.CARTESIAN)),(new Point(71.3, 0,Point.CARTESIAN)),(new Point(56.6, -33,Point.CARTESIAN)),(new Point(23, -38,Point.CARTESIAN))));
        toBackboardFromStack.setConstantHeadingInterpolation(Math.toRadians(-90));
        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new followPath(toSpike),
                        new pivotToTransferPosition(),
                        new WaitCommand(80),
                        new pitchToTransferPosition(),
                        new WaitCommand(400),
                        new grabRightPixel(),
                        new grabLeftPixel(),
                        new WaitUntilCommand(busy),
                        new pitchToSpikePosition(),
                        new WaitCommand(100),
                        new pivotToSpikePosition(),
                        new WaitCommand(300),
                        new releaseLeftPixel(),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                            new followPath(toBackboard),
                            new pitchToDropPosition(),
                            new pivotToDropPosition()
                        ),
                        new WaitUntilCommand(busy),
                        new WaitCommand(500),
                        new releaseRightPixel(),
                        new pivotToWaitPosition(),
                        new pitchToWaitPosition(),
                        new followPath(toStack),
                        new v4BarToHeight(5),
                        new outtakeCommand(),
                        new WaitUntilCommand(busy),
                        new WaitCommand(1000),
                        new intakeCommand(),
                        new WaitCommand(200),
                        new stopIntake(),
                        new WaitCommand(400),
                        new pivotToTransferPosition(),
                        new WaitCommand(200),
                        new pitchToTransferPosition(),
                        new followPath(toBackboardFromStack),
                        new WaitCommand(500),
                        new grabLeftPixel(),
                        new grabRightPixel(),
                        new WaitUntilCommand(busy),
                        new pitchToDropPosition(),
                        new WaitCommand(80),
                        new pivotToDropPosition(),
                        new slideToRow(3),
                        new WaitCommand(1000),
                        new releaseRightPixel(),
                        new releaseLeftPixel()


                )
        );

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
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