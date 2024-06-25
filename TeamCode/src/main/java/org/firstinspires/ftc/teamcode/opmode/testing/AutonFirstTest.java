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

    private boolean purpDrop = true;
    private boolean startCycles = false;
    private boolean backboard = false;
    private boolean stack = false;
    private boolean park = false;

    private Path toPurpDrop;
    private Path toStack;
    private Path toBackboard;
    private Path toPark;


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        toPurpDrop = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(0,30, Point.CARTESIAN)));
        toPurpDrop.setConstantHeadingInterpolation(0);
        toStack = new Path(new BezierLine(new Point(0,30, Point.CARTESIAN), new Point(30,30, Point.CARTESIAN)));
        toStack.setConstantHeadingInterpolation(270);

        CommandScheduler.getInstance().reset();
        CenterstageConstants.IS_AUTO = true;
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);
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
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();

        if (!robot.follower.isBusy()) {
            if (purpDrop) {
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new pitchToDropPosition(),
                                new WaitCommand(1000),
                                new grabLeftPixel(),
                                new grabRightPixel()
                        )
                );
                purpDrop = false;
                startCycles = true;
            }
            else if (startCycles){
                CommandScheduler.getInstance().schedule(new followPath(toStack));
//                CommandScheduler.getInstance().schedule(new followPath(to));
//                for (int i=0; i<3; i++) {
//                    if (!robot.follower.isBusy() && )
//                        robot.follower.followPath(toBackboard);

//                }
                startCycles=false;
//                follower.followPath(toPark);
//                park=true;
            }
            else if (park){
                park=false;
            }
        }
//        follower.telemetryDebug(telemetryA);
    }
}
