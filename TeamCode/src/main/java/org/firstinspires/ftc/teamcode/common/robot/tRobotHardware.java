package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.pathing.localization.FusionLocalizer;
import org.firstinspires.ftc.teamcode.common.pathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.droneSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.followerSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JActuator;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import javax.annotation.concurrent.GuardedBy;

public class tRobotHardware {

    // FINISH THE PERiODIC SUPPLIER CODE FOR CURRENT SENSORS AND CONTROL HUB BaTTERY WITH PHOTONCORE FIX THE ERROR

    private HardwareMap hardwareMap;
    private static tRobotHardware instance = null;
    private boolean enabled;

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

    public intakeSubsystem intake;
    public depositSubsystem deposit;
    public followerSubsystem follower;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ArrayList<Subsystem> subsystems;

    public droneSubsystem drone;

    //    public PreloadDetectionPipeline preloadDetectionPipeline;
    public PropDetectionPipeline propDetectionPipeline;
    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;

//    public PhotonLynxVoltageSensor battery;

    public VoltageSensor battery;
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double startOffset = 0;
    public FusionLocalizer localizer;
    public PoseUpdater poseUpdater;

    private double startTime;
    public HashMap<Sensors.SensorType, Object> values;
    public static tRobotHardware getInstance() {
        if (instance == null) {
            instance = new tRobotHardware();
        }
        instance.enabled = true;
        return instance;
    }
    /*
    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.values = new HashMap<>();

        intake = new intakeSubsystem();
        deposit = new depositSubsystem();
        drone = new droneSubsystem();

        //battery = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
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

        localizer = new FusionLocalizer();
        poseUpdater = new PoseUpdater(localizer);

        leftSlide = new JActuator(leftSlideMotor);
        leftSlide.setPIDController(robotConstants.slideController);
        leftSlide.setFeedforward(JActuator.FeedforwardMode.CONSTANT, robotConstants.slideFF);
        leftSlide.setErrorTolerance(5);
        rightSlide = new JActuator(rightSlideMotor);
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

        roll.setAngularRange(0,Math.toRadians(0),0.56,Math.toRadians(180));
        leftPitch.setAngularRange(0,Math.toRadians(4),0.1,Math.toRadians(-107));
        rightPitch.setAngularRange(0,Math.toRadians(4),0.1,Math.toRadians(-107));
        pivot.setAngularRange(0.56,0,0.86,Math.toRadians(60));

        values.put(Sensors.SensorType.SLIDE_ENCODER, (leftSlideMotor.getCurrentPosition()+rightSlideMotor.getCurrentPosition())/2);
        values.put(Sensors.SensorType.SLIDE_LIMIT, (leftLimit.isPressed() || rightLimit.isPressed()));
        values.put(Sensors.SensorType.POD_PAR0, par0.getPositionAndVelocity());
        values.put(Sensors.SensorType.POD_PAR1, par1.getPositionAndVelocity());
        values.put(Sensors.SensorType.POD_PERP, perp.getPositionAndVelocity());
        values.put(Sensors.SensorType.INTAKE_VELOCITY, Math.abs(intakeRoller.getVelocity()));
        values.put(Sensors.SensorType.LEFT_DISTANCE, USLeft.getDistance());
        values.put(Sensors.SensorType.RIGHT_DISTANCE, USRight.getDistance());
        values.put(Sensors.SensorType.BACK_DISTANCE, USBack.getDistance());
        values.put(Sensors.SensorType.INTAKE_CURRENT, intakeRoller.isOverCurrent());

        // non bulk read
//        values.put(Sensors.SensorType.BATTERY, battery.getCachedVoltage());


        values.put(Sensors.SensorType.BATTERY, battery.getVoltage());
        modules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) CONTROL_HUB = m;
        }

        propDetectionPipeline = new PropDetectionPipeline(0.3, 0.7, 0.65);
        if (CenterstageConstants.IS_AUTO) {

            // TODO: Add start camera here
            synchronized (imuLock) {
                imu = hardwareMap.get(IMU.class, "imu");

                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
                imu.initialize(parameters);
            }

            imuOffset = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }
    }
    */
}