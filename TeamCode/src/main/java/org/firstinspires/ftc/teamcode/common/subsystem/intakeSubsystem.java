package org.firstinspires.ftc.teamcode.common.subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotConstants;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JSubsystem;

import java.util.ArrayList;

public class intakeSubsystem extends JSubsystem {
    robotHardware robot = robotHardware.getInstance();
    private double rollerVelocity;
    private boolean isOverCurrentLimit;
    private boolean isReversed = false;
    private double rollerPower = 0;
    private boolean issueColorSensorCheck = true;
    private double lastCheckTime = 0;
    private boolean isLeftPixel = false;
    private boolean isRightPixel = false;
    private double v4BarAngle = 90;
    public enum intakeState{
        intaking,
        outtaking,
        stationary
    }
    public intakeSubsystem() {

    }

    @Override
    public void read(){
        robot.doubleSubscriber(Sensors.SensorType.INTAKE_VELOCITY);
        robot.boolSubscriber(Sensors.SensorType.INTAKE_CURRENT);
        if(issueColorSensorCheck && (robot.getTimeMs()-lastCheckTime>100)){
            lastCheckTime = robot.getTimeMs();
            isLeftPixel = (robot.leftColorSensor.getDistance(DistanceUnit.MM)>50);
            isRightPixel = (robot.rightColorSensor.getDistance(DistanceUnit.MM)>50);
            issueColorSensorCheck = false;
        }
    }
    @Override
    public void periodic() {
        if((isOverCurrentLimit) || (rollerVelocity < robotConstants.velocityLimit)){
            issueColorSensorCheck = true;
        }
    }
    @Override
    public void write() {
        if(isReversed){robot.intakeRoller.setPower(rollerPower*(-1));}
        else{robot.intakeRoller.setPower(rollerPower);}
    }
    @Override
    public void reset() {
        rollerPower = 0;
        isReversed = false;
    }
    public boolean isReversed(){
        return isReversed;
    }
    public void setMotorDirectionReverse(){
        isReversed = true;
    }
    public void startIntakeRollers(){
        rollerPower = 0.8;
    }
    public void stopIntakeRollers(){
        rollerPower = 0;
    }
    public boolean getLeftPixel(){
        return isLeftPixel;
    }
    public boolean getRightPixel(){
        return isRightPixel;
    }

}
