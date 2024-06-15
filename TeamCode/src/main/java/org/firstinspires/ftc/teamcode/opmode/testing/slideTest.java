package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armRowTenTest;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToRearrangePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.setRollAngle;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.slideToPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.slideToRow;
import org.firstinspires.ftc.teamcode.common.commands.droneLaunch;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.outtakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.stopIntake;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JActuator;

@Config
@TeleOp
public class slideTest extends LinearOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    public static int position=0;



    @Override
    public void runOpMode() throws InterruptedException {

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
            while (opModeInInit()) {
                telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                telemetry.addLine("Robot Initialized.");
                telemetry.update();
            }

        }

        while (!isStopRequested()) {
//            CommandScheduler.getInstance().run();
            robot.read();
            robot.deposit.setTargetPosition(position);
            robot.periodic();
            robot.write();

            telemetry.update();
            telemetry.addData("position.",robot.lift.getPosition());
            telemetry.addData("target",position);
        }

    }


}
