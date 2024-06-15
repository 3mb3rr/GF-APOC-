package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armRowTenTest;
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
import org.firstinspires.ftc.teamcode.common.pathing.follower.Follower;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JActuator;


@TeleOp(name = "teleop coz neev too smart", group = "ykwsp")
public class teleopLinearEnhancedDrivetrain extends LinearOpMode {
    private Follower follower;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private Vector driveVector;
    private Vector headingVector;
    //^^neevs stuff^^


    private final robotHardware robot = robotHardware.getInstance();
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    private double currentTime=0;
    private double lastTime = 0.0;
    private double loopTime=0;

    private double[] rollAngles = {0, Math.toRadians(60), Math.toRadians(120), Math.toRadians(180), Math.toRadians(210), Math.toRadians(300)};
    private int rollIndex = 0;
    private int targetRow = 1;
    private boolean isLeftDropped = false;
    private boolean isRightDropped = false;
    private boolean transferred = false;
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
            driveVector = new Vector();
            headingVector = new Vector();
            while (opModeInInit()) {
                telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                telemetry.addLine("Robot Initialized.");
                telemetry.update();
            }

        }

        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            CommandScheduler.getInstance().run();
            robot.read();
            robot.periodic();
            robot.write();



            gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(this::incrementRollLeft));
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(this::incrementRollRight));

            gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(new Runnable() {
                @Override
                public void run() {
                    increaseSlideRow();
                }
            }));
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(new Runnable() {
                @Override
                public void run() {
                    decreaseSlideRow();
                }
            }));

            if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {CommandScheduler.getInstance().schedule(new releaseLeftPixel()); isLeftDropped = true;}
            if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {CommandScheduler.getInstance().schedule(new releaseRightPixel()); isRightDropped = true;}
            if(isLeftDropped && isRightDropped && (robot.deposit.getArmState() != depositSubsystem.armState.wait)){
                isLeftDropped = false;
                isRightDropped = false;
                transferred = false;
                rollIndex =0;
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                        new slideToRow(1), new armToWaitPosition(), new setRollAngle(0)
                ));

            }
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.X).whenPressed(new intakeCommand());
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new outtakeCommand());
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                    new SequentialCommandGroup(new stopIntake(), new armToTransferPosition(), new WaitCommand(500), new grabLeftPixel(), new grabRightPixel())
            );
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new armRowTenTest());
            if(robot.intake.getLeftPixel() && robot.intake.getRightPixel() && !transferred){
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new stopIntake(), new armToTransferPosition(), new WaitCommand(500), new grabLeftPixel(), new grabRightPixel()));
                transferred = true;
            }
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new SequentialCommandGroup(
                    new setRollAngle(rollAngles[rollIndex]), new armToRearrangePosition(), new slideToRow(targetRow)
            ));
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new SequentialCommandGroup(
                    new setRollAngle(rollAngles[rollIndex]), new armToDropPosition(), new slideToRow(targetRow)
            ));

            gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.X).whenPressed(new droneLaunch());

            currentTime=System.nanoTime();
            loopTime=currentTime - lastTime;
            lastTime = currentTime;

            telemetry.addData("looptime",loopTime);

            telemetry.addData("left position",robot.lift.getPosition());
            telemetry.addData("target",robot.deposit.slideTargetPosition);

            telemetry.addData("voltage",robot.doubleSubscriber(Sensors.SensorType.BATTERY));

            telemetry.update();
        }

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
