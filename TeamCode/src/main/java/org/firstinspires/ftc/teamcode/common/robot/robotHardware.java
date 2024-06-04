package org.firstinspires.ftc.teamcode.common.robot;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.common.pathing.follower.followerSubsystem;
import org.firstinspires.ftc.teamcode.common.pathing.localization.FusionLocalizer;
import org.firstinspires.ftc.teamcode.common.pathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JActuator;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import javax.annotation.concurrent.GuardedBy;


public class robotHardware {

    // FINISH THE PERiODIC SUPPLIER CODE FOR CURRENT SENSORS AND CONTROL HUB BaTTERY WITH PHOTONCORE FIX THE ERROR


    private HardwareMap hardwareMap;
    private static robotHardware instance = null;
    private boolean enabled;

    public JServo v4Bar, transferFlap, leftPitch, rightPitch, pivot, roll;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public RevColorSensorV3 leftColorSensor, rightColorSensor;
    public Encoder par0, par1, perp, leftSlideEnc, rightSlideEnc;
    public AnalogDistanceSensor USLeft, USRight, USBack;
    private List<DcMotorEx> driveMotors;
    public DcMotorEx intakeRoller, leftSlideMotor, rightSlideMotor;
    public JActuator leftSlide;
    public JActuator rightSlide;
    public LimitSwitch leftLimit, rightLimit;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ArrayList<Subsystem> subsystems;

//    public intakeSubsystem intake;
//    public depositSubsystem extension;
    public followerSubsystem follower;

//    public PreloadDetectionPipeline preloadDetectionPipeline;
    public PropDetectionPipeline propDetectionPipeline;
    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;

    VoltageSensor battery;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double startOffset = 0;
    public FusionLocalizer localizer;
    public PoseUpdater poseUpdater = new PoseUpdater(localizer);

    private double startTime;
    public HashMap<Sensors.SensorType, Object> values;



    // TODO: FINISH THE PERiODIC SUPPLIER CODE FOR CURRENT SENSORS AND CONTROL HUB BaTTERY WITH PHOTONCORE FIX THE ERROR
    public static robotHardware getInstance() {
        if (instance == null) {
            instance = new robotHardware();
        }
        instance.enabled = true;
        return instance;
    }


    // TODO: FINISH THE PERiODIC SUPPLIER CODE FOR CURRENT SENSORS AND CONTROL HUB BaTTERY WITH PHOTONCORE FIX THE ERROR
    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.values = new HashMap<>();

        battery = hardwareMap.voltageSensor.get("Control Hub");

        USLeft = new AnalogDistanceSensor(hardwareMap.get(AnalogInput.class, "USLeft"));
        USRight = new AnalogDistanceSensor(hardwareMap.get(AnalogInput.class, "USRight"));
        USBack = new AnalogDistanceSensor(hardwareMap.get(AnalogInput.class, "USBack"));

        leftLimit = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "slideCloseLeft"));
        rightLimit = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "slideCloseRight"));

        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "csLeft");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "csRight");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "slideLeft");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "slideRight");
        intakeRoller = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        leftSlide = new JActuator(
                () -> doubleSubscriber(Sensors.SensorType.SLIDE_ENCODER), () -> boolSubscriber(Sensors.SensorType.SLIDE_LIMIT), leftSlideMotor);
        leftSlide.setPIDController(robotConstants.slideController);
        leftSlide.setFeedforward(JActuator.FeedforwardMode.CONSTANT, robotConstants.slideFF);
        leftSlide.setErrorTolerance(5);
        rightSlide = new JActuator(
                () -> doubleSubscriber(Sensors.SensorType.SLIDE_ENCODER), () -> boolSubscriber(Sensors.SensorType.SLIDE_LIMIT), leftSlideMotor);
        leftSlide.setPIDController(robotConstants.slideController);
        leftSlide.setFeedforward(JActuator.FeedforwardMode.CONSTANT, robotConstants.slideFF);
        leftSlide.setErrorTolerance(5);

        // TODO: reverse MOTOR directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);

        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightRear")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftRear")));

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
        values.put(Sensors.SensorType.BATTERY, battery.getVoltage());

        modules = hardwareMap.getAll(LynxModule.class);

        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber())) CONTROL_HUB = m;
        }

        propDetectionPipeline = new PropDetectionPipeline(0.3, 0.7, 0.65, PropDetectionPipeline.colour.red);
        if (robotConstants.IS_AUTO) {

            // TODO: Add start camera here
            synchronized (imuLock) {
                imu = hardwareMap.get(IMU.class, "imu");

                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
                imu.initialize(parameters);
            }

            imuOffset = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }
    }

    public void periodic(){
        poseUpdater.periodic();
        follower.periodic();
    }
    public void write() {
        follower.write();
    }
    public void read() {
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
        values.put(Sensors.SensorType.BATTERY, battery.getVoltage());
    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested()) {
                synchronized (imuLock) {
                    imuAngle = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                }
            }
        });
        imuThread.start();
    }
    public void readIMU() {
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getAngle() {
        return imuAngle - imuOffset + startOffset;
    }

    public void setDrivetrainPowers(double[] powers){
        for (int i = 0; i < driveMotors.size(); i++) {
            driveMotors.get(i).setPower(powers[i]);
        }
    }
        public double doubleSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }
    public PositionVelocityPair encoderSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0);
        if (value instanceof PositionVelocityPair) {
            return (PositionVelocityPair) value;
        } else {
            throw new ClassCastException();
        }
    }

    public boolean boolSubscriber(Sensors.SensorType topic) {
        return (boolean) values.getOrDefault(topic, 0);
    }
    public void startCamera() {
        aprilTag = new AprilTagProcessor.Builder()
                // calibrated using 3DF Zephyr 7.021
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(800, 448))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(aprilTag, propDetectionPipeline)
                .enableLiveView(true)
                .build();

        visionPortal.setProcessorEnabled(propDetectionPipeline, true);
    }
    public List<AprilTagDetection> getAprilTagDetections() {
        if (aprilTag != null && localizer != null) return aprilTag.getDetections();
        return null;
    }

    public VisionPortal.CameraState getCameraState() {
        if (visionPortal != null) return visionPortal.getCameraState();
        return null;
    }
    public void closeCamera() {
        if (visionPortal != null) visionPortal.close();
    }
    public void startTimer(){
        startTime = System.nanoTime();
    }
    public double getTimeSec(){
        return ((System.nanoTime()-startTime)/100000000);
    }
    public double getTimeMs(){
        return ((System.nanoTime()-startTime)/1000000);
    }


}