package org.firstinspires.ftc.teamcode.common.util.wrappers;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class JServo implements Servo {

    private Servo servo;
    private double radPerVolt = 0.0;
    private double offset = 0.0;

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
        this.servo.setPosition(position);
    }

    @Override
    public double getPosition() {
        return this.servo.getPosition();
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
        this.servo.setPosition((angle/radPerVolt)+offset);
    }
    public double getAngle(){
        return (getPosition()-offset)*radPerVolt;
    }
}