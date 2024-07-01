package org.firstinspires.ftc.teamcode.opmode.testing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.slideToRow;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;


@TeleOp
public class teleopTester extends LinearOpMode {
    private final robotHardware robot = robotHardware.getInstance();
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    private double currentTime=0;
    private double lastTime = 0.0;
    private double loopTime=0;
    private Vector driveVector;
    private Vector headingVector;

    private double[] rollAngles = {0, Math.toRadians(60), Math.toRadians(120), Math.toRadians(180), Math.toRadians(210), Math.toRadians(300)};
    private int rollIndex = 0;
    private int targetRow = 1;
    private boolean isLeftDropped = false;
    private boolean isRightDropped = false;
    private boolean transferred = false;
    private double triggerL=0;
    private double triggerR=0;
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
                telemetry.addLine("Robot Initialized.");
                telemetry.update();
            }

        }

        while (!isStopRequested()) {
            CommandScheduler.getInstance().run();
            robot.read();
            robot.periodic();
            robot.write();
            driveVector.setOrthogonalComponents(-gamepadDrivetrain.getLeftY(), gamepadDrivetrain.getRightY());
            driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
            driveVector.rotateVector(robot.follower.getPose().getHeading());
            headingVector.setComponents((triggerL-triggerR), robot.follower.getPose().getHeading());
            robot.follower.setMovementVectors(robot.follower.getCentripetalForceCorrection(), headingVector, driveVector);
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new slideToRow(8));
            gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new slideToRow(0));
//            CommandScheduler.getInstance().run();
//            robot.read();
//            robot.periodic();
//            robot.write();
//
//            if (gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)<1) triggerL=gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)*0.5;
//            else triggerL=1;
//            if (gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)<1) triggerR=gamepadDrivetrain.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)*0.5;
//            else triggerR=1;
//
//            driveVector.setOrthogonalComponents(-gamepadDrivetrain.getLeftY(), gamepadDrivetrain.getRightY());
//            driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
//            driveVector.rotateVector(robot.follower.getPose().getHeading());
//            headingVector.setComponents((triggerL-triggerR), robot.follower.getPose().getHeading());
//            robot.follower.setMovementVectors(robot.follower.getCentripetalForceCorrection(), headingVector, driveVector);
//
//            gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(this::incrementRollLeft));
//            gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(this::incrementRollRight));
//
//            gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(this::increaseSlideRow));
//            gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(this::decreaseSlideRow));
//
//            if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {CommandScheduler.getInstance().schedule(new releaseLeftPixel()); isLeftDropped = true;}
//            if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {CommandScheduler.getInstance().schedule(new releaseRightPixel()); isRightDropped = true;}
//            if(isLeftDropped && isRightDropped && (robot.deposit.getPivotState() != depositSubsystem.armState.wait) && (robot.deposit.getPitchState() != depositSubsystem.armState.wait)){
//                isLeftDropped = false;
//                isRightDropped = false;
//                transferred = false;
//                rollIndex =0;
////                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
////                        new WaitCommand(200),new slideToRow(0), new WaitCommand(200),new pivotToWaitPosition(),new pitchToWaitPosition(), new setRollAngle(0)
////                ));
//
//            }
//            gamepadMechanism.getGamepadButton(GamepadKeys.Button.X).whenPressed(new intakeCommand());
//            gamepadMechanism.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new outtakeCommand());
//            gamepadMechanism.getGamepadButton(GamepadKeys.Button.A).whenPressed( new stopIntake()
//                    //new SequentialCommandGroup(new stopIntake(), new WaitCommand(250),new pivotToTransferPosition(), new WaitCommand(300),new pitchToTransferPosition(), new WaitCommand(660), new grabLeftPixel(), new grabRightPixel())
//            );
////            if(robot.intake.getLeftPixel() && robot.intake.getRightPixel() && !transferred){
////                CommandScheduler.getInstance().schedule(
////                        new SequentialCommandGroup(
////                                new WaitCommand(200),
////                                new outtakeCommand(),
////                                new WaitCommand(200),
////                                new stopIntake(),
////                                new WaitCommand(250),
////                                new pivotToTransferPosition(),
////                                new WaitCommand(200),
////                                new pitchToTransferPosition(),
////                                new WaitCommand(660),
////                                new grabLeftPixel(),
////                                new grabRightPixel()
////                        )
////                );
////                transferred = true;
////            }
////            gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new SequentialCommandGroup(
////                    new pivotToRearrangePosition(), new pitchToRearrangePosition(), new slideToRow(targetRow)
////            ));
//            if (gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get()) {
//                isLeftDropped = false;
//                isRightDropped = false;
//                CommandScheduler.getInstance().schedule(
////                        new SequentialCommandGroup(
////                                new setRollAngle(rollAngles[0]),
////                                new pitchToDropPosition(),
////                                new WaitCommand(80),
////                                new pivotToDropPosition(),
//                        new slideToRow(targetRow)
////                        )
//                );
//            }
//
//            gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.X).whenPressed(new droneLaunch());
////            gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SequentialCommandGroup(
////                    new slideToRow(8), new intakeToHang(), new pitchToDropPosition(), new pivotToDropPosition()));
//            gamepadDrivetrain.getGamepadButton(GamepadKeys.Button.B).whenPressed(new hangCommand());

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
