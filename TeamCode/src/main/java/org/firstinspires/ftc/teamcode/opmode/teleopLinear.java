package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
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
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.intakeSubsystem;


@TeleOp
public class teleopLinear extends LinearOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    private double loopTime = 0.0;
    private Vector driveVector;
    private Vector headingVector;

    @Override
    public void runOpMode() throws InterruptedException {

        while (!isStopRequested() && !isStarted()){
            CommandScheduler.getInstance().reset();
            CenterstageConstants.IS_AUTO = false;
            gamepadDrivetrain = new GamepadEx(gamepad1);
            gamepadMechanism = new GamepadEx(gamepad2);
            robot.init(hardwareMap);
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

            driveVector.setOrthogonalComponents(-gamepadDrivetrain.getLeftY(), -gamepadDrivetrain.getLeftX());
            driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
            driveVector.rotateVector(robot.follower.getPose().getHeading());
            headingVector.setComponents((gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), robot.follower.getPose().getHeading());
            robot.follower.setMovementVectors(robot.follower.getCentripetalForceCorrection(), headingVector, driveVector);

            gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(this::incrementRollLeft));
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(this::incrementRollRight));
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(this::increaseSlideRow));
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(this::decreaseSlideRow));
            if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
                CommandScheduler.getInstance().schedule(new releaseLeftPixel());
            if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
                CommandScheduler.getInstance().schedule(new releaseRightPixel());

            gamepadMechanism.getGamepadButton(GamepadKeys.Button.X).whenPressed(new intakeCommand());
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new outtakeCommand());
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                    new SequentialCommandGroup(new stopIntake(), new armToTransferPosition(), new grabLeftPixel(), new grabRightPixel())
            );
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new armToRearrangePosition());
            if (robot.intake.getLeftPixel() && robot.intake.getRightPixel())
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(new stopIntake(), new armToTransferPosition(), new grabLeftPixel(), new grabRightPixel())
                );

            gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.X).whenPressed(new droneLaunch());

            telemetry.addData("left pixel", robot.intake.getLeftPixel());
            telemetry.addData("right pixel", robot.intake.getRightPixel());
            telemetry.update();
        }

        }
        public void incrementRollLeft () {
            if (robot.deposit.getRollIndex() != 0)
                robot.deposit.setRollIndex(robot.deposit.getRollIndex() - 1);
            else robot.deposit.setRollIndex(5);
        }

        public void incrementRollRight () {
            if (robot.deposit.getRollIndex() != 5)
                robot.deposit.setRollIndex(robot.deposit.getRollIndex() + 1);
            else robot.deposit.setRollIndex(0);
        }

        public void decreaseSlideRow () {
            if (robot.deposit.getSlideTargetRow() != 0)
                robot.deposit.setSlideTargetRow(robot.deposit.getSlideTargetRow() - 1);
            else robot.deposit.setSlideTargetRow(0);
        }
        public void increaseSlideRow () {
            robot.deposit.setSlideTargetRow(robot.deposit.getSlideTargetRow() + 1);
        }


}
