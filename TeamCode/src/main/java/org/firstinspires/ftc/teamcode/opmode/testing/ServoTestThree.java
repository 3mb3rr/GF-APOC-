package org.firstinspires.ftc.teamcode.opmode.testing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToRearrangePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.droneLaunch;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.outtakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.stopIntake;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.v4BarToHeight;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;



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
                telemetry.addLine("Robot Initialized.");
                telemetry.update();
            }

        }

        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.periodic();
            robot.write();
            if (gamepad2.x) {
                telemetry.addLine("im straight");
                CommandScheduler.getInstance().schedule(new v4BarToHeight(i));
            }
            if (gamepad2.y) {
                telemetry.addLine("im lesbian");
                i = i + 1;
                CommandScheduler.getInstance().schedule(new v4BarToHeight(i));
            }
            if (gamepad2.a) {
                i = i - 1;
                telemetry.addLine("im gay"); //aryan set these telemetry statements not me wtf is this
                CommandScheduler.getInstance().schedule(new v4BarToHeight(i));
            }

            telemetry.update();
        }
    }
}
