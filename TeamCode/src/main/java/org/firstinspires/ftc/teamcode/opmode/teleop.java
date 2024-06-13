package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToDropPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToRearrangePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToWaitPosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.setRollAngle;
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
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;


@TeleOp
public class teleop extends CommandOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    private double loopTime = 0.0;
    private Vector driveVector;
    private Vector headingVector;

    private double[] rollAngles = {0, Math.toRadians(30), Math.toRadians(150), Math.toRadians(180), Math.toRadians(210), Math.toRadians(270)};
    private int rollIndex = 0;
    private int targetRow = robot.deposit.getSlideTargetRow();
    private boolean isLeftDropped = false;
    private boolean isRightDropped = false;
    @Override
    public void initialize(){
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
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
        robot.read();
        robot.periodic();
        robot.write();

        driveVector.setOrthogonalComponents(-gamepadDrivetrain.getLeftY(), -gamepadDrivetrain.getLeftX());
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(robot.follower.getPose().getHeading());
        headingVector.setComponents((gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)-gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), robot.follower.getPose().getHeading());
        robot.follower.setMovementVectors(robot.follower.getCentripetalForceCorrection(), headingVector, driveVector);

        gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(this::incrementRollLeft));
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(this::incrementRollRight));

        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(this::increaseSlideRow));
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(this::decreaseSlideRow));

        if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {CommandScheduler.getInstance().schedule(new releaseLeftPixel()); isLeftDropped = true;}
        if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {CommandScheduler.getInstance().schedule(new releaseRightPixel()); isRightDropped = true;}
        if(isLeftDropped && isRightDropped && (robot.deposit.getArmState() != depositSubsystem.armState.wait)){
            CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new slideToRow(0), new armToWaitPosition(), new setRollAngle(0)
            ));
        }
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.X).whenPressed(new intakeCommand());
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new outtakeCommand());
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(new stopIntake(), new armToTransferPosition(), new grabLeftPixel(), new grabRightPixel())
        );
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new SequentialCommandGroup(
                new setRollAngle(rollAngles[rollIndex]), new armToRearrangePosition(), new slideToRow(targetRow)
        ));
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new SequentialCommandGroup(
            new setRollAngle(rollAngles[rollIndex]), new armToDropPosition(), new slideToRow(targetRow)
        ));

        gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.X).whenPressed(new droneLaunch());


    }
    public void incrementRollLeft(){
        if(rollIndex != 0) rollIndex-=1;
        else rollIndex = 5;
    }

    public void incrementRollRight(){
        if(rollIndex != 5) rollIndex+=1;
        else rollIndex = 0;
    }

    public void decreaseSlideRow(){
        targetRow = robot.deposit.getSlideTargetRow();
        if(targetRow != 0) targetRow-=1;
        else targetRow = 0;
    }
    public void increaseSlideRow(){
        targetRow = robot.deposit.getSlideTargetRow();
        targetRow+=1;
    }


}
