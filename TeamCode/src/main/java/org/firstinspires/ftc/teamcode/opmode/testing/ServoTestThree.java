package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;


@Autonomous
public class ServoTestThree extends LinearOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    private double loopTime = 0.0;
    private Vector driveVector;
    private Vector headingVector;

    private double[] rollAngles = {0, Math.toRadians(60), Math.toRadians(120), Math.toRadians(180), Math.toRadians(210), Math.toRadians(300)};
    private int rollIndex = 0;
    private int targetRow = 1;
    private boolean isLeftDropped = false;
    private boolean isRightDropped = false;
    @Override
    public void runOpMode() throws InterruptedException {
        int i=1;
        while (!isStopRequested() && !isStarted()){
            CommandScheduler.getInstance().reset();
            CenterstageConstants.IS_AUTO = false;
            gamepadDrivetrain = new GamepadEx(gamepad1);
            gamepadMechanism = new GamepadEx(gamepad2);
            robot.init(hardwareMap);
            robot.follower.setAuto(CenterstageConstants.IS_AUTO);
            robot.read();
            robot.periodic();
            robot.write();
            driveVector = new Vector();
            headingVector = new Vector();
            while (opModeInInit()) {
                telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                telemetry.addLine("Robot Initialized.");
                telemetry.update();
                robot.rightPitch.setAngle(Math.toRadians(4));
                robot.leftPitch.setAngle(Math.toRadians(4));
                robot.roll.setAngle(0);
                robot.pivot.setAngle(Math.toRadians(-105));
                robot.v4Bar.setPosition(1);
                robot.transferFlap.setPosition(0);
            }

        }

        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.periodic();
            robot.write();

            if (gamepad2.x) {
                robot.rightPitch.setAngle(Math.toRadians(-110));
                robot.leftPitch.setAngle(Math.toRadians(-110));
            }
            if (gamepad2.y) {
                robot.rightPitch.setAngle(Math.toRadians(4));
                robot.leftPitch.setAngle(Math.toRadians(4));
            }
            if (gamepad2.a) {
                robot.rightPitch.setAngle(Math.toRadians(34));
                robot.leftPitch.setAngle(Math.toRadians(34));
                robot.pivot.setAngle(56);
            }
            telemetry.addData("fourbar angle",robot.intake.v4BarAngle);
            telemetry.addData("fourbar position",robot.v4Bar.getPosition());
            telemetry.update();
        }
    }
}
