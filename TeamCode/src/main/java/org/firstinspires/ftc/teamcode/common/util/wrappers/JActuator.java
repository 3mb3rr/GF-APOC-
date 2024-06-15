package org.firstinspires.ftc.teamcode.common.util.wrappers;

import static org.firstinspires.ftc.teamcode.common.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class JActuator {
    public enum FeedforwardMode {
        NONE,
        CONSTANT,
        ANGLE_BASED,
        ANGLE_BASED_SIN
    }
    private robotHardware robot = robotHardware.getInstance();
    private final Map<String, HardwareDevice> devices = new HashMap<>();
    private PIDController controller;
    private DoubleSupplier voltage;

    private double position = 0.0;
    private double targetPosition = 0.0;
    private double pPower = 0.0;
    private double power = 0.0;
    private double tolerance = 0.0;
    private double feedforwardMin = 0.0;
    private double feedforwardMax = 0.0;
    private double currentFeedforward = 0.0;
    private double targetPositionOffset = 0.0;
    public boolean digitalResetValue;

    private boolean reached = false;
    private boolean floating = false;
    private boolean issueEncoderReset = false;

    private FeedforwardMode mode = FeedforwardMode.NONE;

    private Sensors.SensorType sensorType;
    private Supplier<Object> topic1, topic2;

    /**
     * Actuator constructor with varargs HardwareDevice parameter
     *
     * @param
     */
    public JActuator(){

    }
    public JActuator(HardwareDevice... devices) {
        this.topic1 = null;
        this.topic2 = null;
        int i = 0;
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName() + " " + i++, device);
        }
        read();
    }

    public JActuator(Supplier<Object> topic1, Supplier<Object> topic2, HardwareDevice... devices) {
        this.topic1 = topic1;
        this.topic2 = topic2;
        int i = 0;
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName() + " " + i++, device);
        }
        read();
    }
    public JActuator(Supplier<Object> topic1, HardwareDevice... devices) {
        this.topic1 = topic1;
        this.topic2 = null;
        int i = 0;
        for (HardwareDevice device : devices) {
            this.devices.put(device.getDeviceName() + " " + i++, device);
        }
        read();
    }

    /**
     * Reads every given hardware device containing either AnalogEncoder or Encoder object.
     * Will then store this, and terminate the loop, because there is only a need for one value in
     * a given actuation group.
     */
    public void read() {

        if (topic1 != null) {
            Object value = topic1.get();
            if (value instanceof Integer) {
                this.position = (int) value;
                return;
            } else if (value instanceof Double) {
                this.position = (double) value;
                return;
            }
        }
        if (topic2 != null) {
            Object value = topic2.get();
            if (value instanceof Boolean) {
                this.digitalResetValue = (boolean) value;
                return;}
        }
        this.position = 0.0;
    }

    /**
     * Performs arithmetic with the AsymmetricMotionProfile and PIDController.
     * Stores a boolean representing whether or not the actuator group is within
     * some tolerance given by a specified value.
     */
    public void periodic() {
        if(this.digitalResetValue==true){
            issueEncoderReset = true;
        }
        if (controller != null) {
            this.power = controller.calculate(position, targetPosition + targetPositionOffset);

            switch (mode) {
                case CONSTANT:
                    this.power += currentFeedforward;
                    break;
                case ANGLE_BASED:
                    this.power -= Math.cos(targetPosition + targetPositionOffset) * currentFeedforward;
                    break;
                case ANGLE_BASED_SIN:
                    this.power -= Math.sin(targetPosition + targetPositionOffset) * currentFeedforward;
                    break;
                default:
            }
            this.power = clamp(power, -1, 1);
        }

        this.reached = Math.abs((targetPosition + targetPositionOffset) - position) < tolerance;
    }

    /**
     * For cohesiveness in our program's sequence, writes back all of the new
     * values calculated and saved. Runs different methods based on the given actuation group.
     */
    public void write() {

        if(issueEncoderReset){for (HardwareDevice device : devices.values()) {
            if (device instanceof DcMotor) {
                ((DcMotor) device).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                ((DcMotor) device).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                issueEncoderReset = false;
            }
        }}
        if (Math.abs(power - pPower) > 0.005) {
            for (HardwareDevice device : devices.values()) {
                if (device instanceof DcMotor) {
                    double correction = 1.0;
                    if (voltage != null) correction = 12.0 / voltage.getAsDouble();
                    if (!floating) ((DcMotor) device).setPower(power * correction);
                    else ((DcMotor) device).setPower(0);
                    pPower = power;
                }
            }
        }
    }


    /**
     * Will set the target position for the actuator group. In the case of a motion profile
     * being used, the profile will be reset and created again with the new target position.
     * Otherwise, a PIDController will be used in the periodic() method above.
     *
     * @param targetPosition
     */
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void setFloat(boolean f) {
        this.floating = f;
    }





    public void setVoltageSupplier(DoubleSupplier voltage) {
        this.voltage = voltage;
    }

    public void setPIDController(PIDController controller) {
        this.controller = controller;
    }

    public void setCurrentPosition(double position) {
        this.position = position;

    }

    public void setPID(double p, double i, double d) {
        if (controller == null) {
            this.controller = new PIDController(p, i, d);
        } else {
            this.controller.setPID(p, i, d);
        }

    }

    public void setFeedforward(FeedforwardMode mode, double feedforward) {
        this.mode = mode;
        this.feedforwardMin = feedforward;
        this.currentFeedforward = feedforwardMin;
    }

    public void setFeedforward(FeedforwardMode mode, double feedforwardMin, double feedforwardMax) {
        this.mode = mode;
        this.feedforwardMin = feedforwardMin;
        this.feedforwardMax = feedforwardMax;
        this.currentFeedforward = feedforwardMin;
    }

    /**
     * Sets the allowed error tolerance for the actuation group to be considered
     * "close enough" within the target position.
     *
     * @param tolerance
     * @return
     */
    public void setErrorTolerance(double tolerance) {
        this.tolerance = tolerance;
    }


    public void updatePID(double P, double I, double D) {
        this.controller.setPID(P, I, D);
    }

    public void updateFeedforward(double ff) {
        this.currentFeedforward = ff;
    }

    /**
     * Gets the value read by the actuation group.
     *
     * @return double
     */
    public double getPosition() {
        return position;
    }

    /**
     * Gets the current target position for the actuation group.
     *
     * @return double
     */
    public double getTargetPosition() {
        return targetPosition;
    }


    public double getPower() {
        return power;
    }

    public double getCurrentFeedforward() {
        return currentFeedforward;
    }

    /**
     * Gets any given HardwareDevice within this actuator group.
     *
     * @param deviceName The given HardwareMap name specified in the config,
     *                   or specified at object creation.
     * @return HardwareDevice object
     */
    public HardwareDevice getDevice(String deviceName) {
        return this.devices.get(deviceName);
    }

    /**
     * Returns a list of all given HardwareDevices.
     *
     * @return
     */
    public List<HardwareDevice> getDevices() {
        return new ArrayList<>(devices.values());
    }


    /**
     * Returns whether or not the given actuation group is within error
     * tolerance of the final position.
     *
     * @return
     */
    public boolean hasReached() {
        return this.reached;
    }

}