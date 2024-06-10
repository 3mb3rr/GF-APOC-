package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToRearrangePosition;
import org.firstinspires.ftc.teamcode.common.commands.armCommands.armToTransferPosition;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.grabRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseLeftPixel;
import org.firstinspires.ftc.teamcode.common.commands.dropperCommands.releaseRightPixel;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.intakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.outtakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.intakeCommands.stopIntake;
import org.firstinspires.ftc.teamcode.common.robot.AnalogDistanceSensor;
import org.firstinspires.ftc.teamcode.common.robot.LimitSwitch;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotConstants;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JActuator;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;

import java.util.Arrays;
import java.util.List;

@Autonomous
public class tester extends LinearOpMode {
    private GamepadEx gamepadDrivetrain;
    private GamepadEx gamepadMechanism;
    public JServo v4Bar, transferFlap, leftPitch, rightPitch, pivot, roll, fingerLeft, fingerRight, droneServo;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public RevColorSensorV3 leftColorSensor, rightColorSensor;
    public Encoder par0, par1, perp;
    public AnalogDistanceSensor USLeft, USRight, USBack;
    private List<DcMotorEx> driveMotors;
    public DcMotorEx intakeRoller, leftSlideMotor, rightSlideMotor;
    public JActuator leftSlide;
    public JActuator rightSlide;
    public LimitSwitch leftLimit, rightLimit;
    public VoltageSensor battery;

    @Override
    public void runOpMode(){
        gamepadDrivetrain = new GamepadEx(gamepad1);
        gamepadMechanism = new GamepadEx(gamepad2);

        battery = hardwareMap.voltageSensor.get("Control Hub");
        USLeft = new AnalogDistanceSensor(hardwareMap.get(AnalogInput.class, "USLeft"));
        USRight = new AnalogDistanceSensor(hardwareMap.get(AnalogInput.class, "USRight"));
        USBack = new AnalogDistanceSensor(hardwareMap.get(AnalogInput.class, "USBack"));

        leftLimit = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "slideCloseLeft"));
        rightLimit = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "slideCloseRight"));

        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "CSLeft");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "CSRight");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "slideLeft");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "slideRight");
        intakeRoller = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        transferFlap = new JServo(hardwareMap.get(Servo.class, "flapServo"));
        v4Bar = new JServo(hardwareMap.get(Servo.class, "fourBarServo"));
        fingerRight = new JServo(hardwareMap.get(Servo.class, "fingerServoRight"));
        fingerLeft = new JServo(hardwareMap.get(Servo.class, "fingerServoLeft"));
        roll = new JServo(hardwareMap.get(Servo.class, "wristServo"));
        pivot = new JServo(hardwareMap.get(Servo.class, "pivotServo"));
        leftPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoLeft"));
        rightPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoRight"));
        droneServo = new JServo(hardwareMap.get(Servo.class, "droneServo"));
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));


        leftSlide = new JActuator(leftSlideMotor);
        leftSlide.setPIDController(robotConstants.slideController);
        leftSlide.setFeedforward(JActuator.FeedforwardMode.CONSTANT, robotConstants.slideFF);
        leftSlide.setErrorTolerance(5);
        rightSlide = new JActuator(leftSlideMotor);
        leftSlide.setPIDController(robotConstants.slideController);
        leftSlide.setFeedforward(JActuator.FeedforwardMode.CONSTANT, robotConstants.slideFF);
        leftSlide.setErrorTolerance(5);

        // TODO: reverse MOTOR directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));

        // TODO: reverse encoder directions if needed
        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        // TODO: set motor modes
        for (DcMotorEx motor : driveMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        intakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRoller.setCurrentAlert(robotConstants.currentLimit, CurrentUnit.AMPS);

        waitForStart();

        while (!isStopRequested()){
//                    gamepadMechanism.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(this::incrementRollLeft));
//        gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(this::incrementRollRight));
//        gamepadMechanism.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(this::increaseSlideRow));
//        gamepadMechanism.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(this::decreaseSlideRow));
        if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) CommandScheduler.getInstance().schedule(new releaseLeftPixel());
        if (gamepadMechanism.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) CommandScheduler.getInstance().schedule(new releaseRightPixel());

        gamepadMechanism.getGamepadButton(GamepadKeys.Button.X).whenPressed(new intakeCommand());
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new outtakeCommand());
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(new stopIntake(), new armToTransferPosition(), new grabLeftPixel(), new grabRightPixel())
        );
        gamepadMechanism.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new armToRearrangePosition());
        }
    }
}
