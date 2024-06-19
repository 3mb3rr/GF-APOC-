package org.firstinspires.ftc.teamcode.common.util.wrappers;

import static java.lang.Double.NaN;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.common.util.MathUtils;

public class JServo implements Servo {

    private Servo servo;
    private double radPerVolt = 0.0;
    private double offset = 0.0;
    private double targetAngle = 0; //NaN
    private double lastPosition = 0; //NaN


    public JServo(Servo servo) {
        this.servo = servo;
    }


    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return servo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    @Override
    public ServoController getController() {
        return servo.getController();
    }

    @Override
    public int getPortNumber() {
        return servo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        this.servo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return this.servo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        lastPosition = MathUtils.clamp(position,0,1);
        this.servo.setPosition(lastPosition);
    }

    @Override
    public double getPosition() {
        return lastPosition;
    }

    @Override
    public void scaleRange(double min, double max) {
        this.servo.scaleRange(min, max);
    }

    public void setAngularRange(double v1, double angle1, double v2, double angle2){
        radPerVolt = (angle2 - angle1)/(v2-v1);
        offset=v1-angle1/radPerVolt;
    }

    public void setAngle(double angle){
        targetAngle = angle;
        this.servo.setPosition(MathUtils.clamp(((targetAngle/radPerVolt)+offset),0,1));
    }
    public double getAngle(){
        return (getPosition()-offset)*radPerVolt;
    }
}