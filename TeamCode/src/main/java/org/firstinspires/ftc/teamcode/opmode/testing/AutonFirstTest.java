//package org.firstinspires.ftc.teamcode.opmode.testing;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.common.pathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
//
//
//@Config
//@Autonomous
//public class AutonFirstTest extends OpMode {
//    private Telemetry telemetryA;
//    private boolean toPurpDrop = true;
//
//    private Follower follower;
//
//    private Path toPurpDrop;
//    private Path backwards;
//
//    /**
//     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
//     * initializes the FTC Dashboard telemetry.
//     */
//    @Override
//    public void init() {
//        follower = new Follower(hardwareMap);
//
//        toPurpDrop = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(DISTANCE,0, Point.CARTESIAN)));
//        forwards.setConstantHeadingInterpolation(0);
//        backwards = new Path(new BezierLine(new Point(DISTANCE,0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));
//        backwards.setConstantHeadingInterpolation(0);
//
//        follower.followPath(forwards);
//
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
//                + " inches forward. The robot will go forward and backward continuously"
//                + " along the path. Make sure you have enough room.");
//        telemetryA.update();
//    }
//
//    /**
//     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
//     * the Telemetry, as well as the FTC Dashboard.
//     */
//    @Override
//    public void loop() {
//        follower.update();
//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.followPath(backwards);
//            } else {
//                forward = true;
//                follower.followPath(forwards);
//            }
//        }
//
//        telemetryA.addData("going forward", forward);
//        follower.telemetryDebug(telemetryA);
//    }
//}
