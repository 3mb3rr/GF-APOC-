package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.pitchToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.followPath;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;


@Config
@Autonomous
public class AutonFirstTest extends OpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private Telemetry telemetryA;

    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        forwards = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(0);


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();

        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);

        robot.follower.followPath(forwards);
//        robot.read();
//        robot.periodic();
//        robot.write();



        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("brorrther ouuhhh");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {


        if (!robot.follower.isBusy()) {
            if (forward) {
                forward = false;
                robot.follower.followPath(backwards);
            } else {
                forward = true;
                robot.follower.followPath(forwards);
            }
        }

        telemetryA.addData("baby do uknow what thats worth", forward);


        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();
        }
    }