package org.firstinspires.ftc.teamcode.common.robot;

import android.util.Size;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.pathing.localization.AprilTagConstants;
import org.firstinspires.ftc.teamcode.common.pathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.pathing.localization.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.common.subsystem.followerSubsystem;
import org.firstinspires.ftc.teamcode.common.pathing.localization.FusionLocalizer;
import org.firstinspires.ftc.teamcode.common.pathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.droneSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.intakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JActuator;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PreloadDetectionPipeline;
import org.firstinspires.ftc.teamcode.common.vision.PropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import javax.annotation.concurrent.GuardedBy;

//@Photon
public class robotHardware {

    // FINISH THE PERiODIC SUPPLIER CODE FOR CURRENT SENSORS AND CONTROL HUB BaTTERY WITH PHOTONCORE FIX THE ERROR

    private HardwareMap hardwareMap;
    private static robotHardware instance = null;
    private boolean enabled;

    public JServo v4Bar, transferFlap, leftPitch, rightPitch, pivot, roll, fingerLeft, fingerRight, droneServo, latch,droneHeight;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public RevColorSensorV3 leftColorSensor, rightColorSensor;
    public Encoder par0, par1, perp;
    public AnalogDistanceSensor USLeft, USRight, USBack;
    public List<DcMotorEx> driveMotors;
    public DcMotorEx intakeRoller, leftSlideMotor, rightSlideMotor;
    public JActuator lift;
    public LimitSwitch leftLimit, rightLimit;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ArrayList<Subsystem> subsystems;

    public intakeSubsystem intake;
    public depositSubsystem deposit;
    public followerSubsystem follower;
    public droneSubsystem drone;

    //    public PreloadDetectionPipeline preloadDetectionPipeline;
    public PropDetectionPipeline propDetectionPipeline;
    public PreloadDetectionPipeline preloadDetectionPipeline;
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
        droneServo = new JServo(hardwareMap.get(Servo.class, "droneLaunchServo"));
        latch = new JServo(hardwareMap.get(Servo.class, "latchServo"));
        droneHeight = new JServo(hardwareMap.get(Servo.class, "droneHeightServo"));

        // TODO: reverse MOTOR directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));

        // TODO: reverse encoder directions if needed
        par0.setDirection(DcMotorEx.Direction.REVERSE);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);

        localizer = new FusionLocalizer();
        poseUpdater = new PoseUpdater(localizer);


        driveMotors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        // TODO: set motor modes
        for (DcMotorEx motor : driveMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intakeRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRoller.setCurrentAlert(robotConstants.currentLimit, CurrentUnit.AMPS);

        lift = new JActuator(() -> leftSlideMotor.getCurrentPosition(), leftSlideMotor, rightSlideMotor);
        lift.setPIDController(new PIDController(0.02, 0, 0.000475));
        lift.setFeedforward(JActuator.FeedforwardMode.CONSTANT, robotConstants.slideFF);
        lift.setErrorTolerance(5);

        roll.setAngularRange(0,Math.toRadians(0),0.56,Math.toRadians(180));
        leftPitch.setAngularRange(0.5,Math.toRadians(0),0.16,Math.toRadians(-96));
        rightPitch.setAngularRange(0.5,Math.toRadians(0),0.16,Math.toRadians(-96));
        pivot.setAngularRange(0.53,0,0.87,Math.toRadians(90));

        transferFlap.setAngularRange(0,0,1,Math.toRadians(90));
        v4Bar.setAngularRange(1,Math.toRadians(75),0.34,Math.toRadians(0));

        intake = new intakeSubsystem();
        deposit = new depositSubsystem();
        drone = new droneSubsystem();
        follower = new followerSubsystem();



//        follower.holdPoint(new BezierPoint(new Point(0, 0, 1)), 0);


//        values.put(Sensors.SensorType.SLIDE_ENCODER, (leftSlideMotor.getCurrentPosition()));
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
        preloadDetectionPipeline = new PreloadDetectionPipeline();
        if (CenterstageConstants.IS_AUTO) {

            // TODO: Add start camera here
            synchronized (imuLock) {
                imu = hardwareMap.get(IMU.class, "imu");

                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
                imu.initialize(parameters);
            }

            imuOffset = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));


            for (DcMotorEx motor : driveMotors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            leftPitch.setAngle(robotConstants.transferPitch);
            rightPitch.setAngle(robotConstants.transferPitch);
            pivot.setAngle(robotConstants.pivotTransferAngle);
            v4Bar.setPosition(1);
            latch.setPosition(robotConstants.latchClose);
            fingerLeft.setPosition(robotConstants.grabPos);
            fingerRight.setPosition(robotConstants.grabPos);
        }
        else {
            leftPitch.setAngle(robotConstants.waitPitch);
            rightPitch.setAngle(robotConstants.waitPitch);
            pivot.setAngle(robotConstants.pivotWaitAngle);
            intake.setV4BarAngle(1);
        }
        roll.setAngle(0);
        transferFlap.setAngle(90);
        droneHeight.setPosition(1);
        droneServo.setPosition(1);

    }

    public void periodic(){
        poseUpdater.periodic();
        follower.periodic();
        intake.periodic();
        deposit.periodic();
        drone.periodic();

    }
    public void write() {
        deposit.write();
        follower.write();
        intake.write();
        drone.write();
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
//        values.put(Sensors.SensorType.BATTERY, battery.getCachedVoltage());
//
        values.put(Sensors.SensorType.BATTERY, battery.getVoltage());

        intake.read();
        deposit.read();
        drone.read();
        follower.read();
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
        preloadDetectionPipeline.setTargetAprilTagID(Location.CENTER);
        aprilTag = new AprilTagProcessor.Builder()
                // calibrated using 3DF Zephyr 7.021
                .setLensIntrinsics(605.988, 605.988, 388.845, 214.577   )
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(800, 448))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessors(propDetectionPipeline) //aprilTag, preloadDetectionPipeline
                .enableLiveView(true)
                .build();
//       - visionPortal.setProcessorEnabled(aprilTag, true);
//        visionPortal.setProcessorEnabled(preloadDetectionPipeline, true);

    }
    public List<AprilTagDetection> getAprilTagDetections() {
        if (aprilTag != null && localizer != null) return aprilTag.getDetections();
        return null;
    }

    public VisionPortal.CameraState getCameraState() {
        if (visionPortal != null) return visionPortal.getCameraState();
        return null;
    }
    public Pose getAprilTagPosition() {
        if (aprilTag != null && localizer != null) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            List<Pose> backdropPositions = new ArrayList<>();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    switch (detection.id) {
                        case 1:
                        case 4:
                            Pose temp1 = new Pose(detection.ftcPose);
                            temp1.add(new Pose(6, 0, 0));
                            backdropPositions.add(temp1);
                            break;
                        case 2:
                        case 5:
                            backdropPositions.add(new Pose(detection.ftcPose));
                            break;
                        case 3:
                        case 6:
                            Pose temp2 = new Pose(detection.ftcPose);
                            temp2.subtract(new Pose(6, 0, 0));
                            backdropPositions.add(temp2);
                            break;
                        default:
                            break;
                    }
                }
            }

            Pose accumulator = new Pose();

            // Manually add each Pose to the accumulator
            for (Pose pose : backdropPositions) {
                accumulator.add(pose);
            }

            // Compute the average position
            Pose backdropPosition = new Pose(accumulator.getX(), accumulator.getY(), accumulator.getHeading());
            backdropPosition.scale(1.0 / backdropPositions.size());


            Pose globalTagPosition = localizer.getPose().getX() < 0 ?
                    AprilTagConstants.convertBlueBackdropPoseToGlobal(backdropPosition) :
                    AprilTagConstants.convertRedBackdropPoseToGlobal(backdropPosition);

            if (Double.isNaN(globalTagPosition.getX()) || Double.isNaN(globalTagPosition.getY()) || Double.isNaN(globalTagPosition.getHeading())) return null;

            return globalTagPosition;
        } else {
            return null;
        }
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