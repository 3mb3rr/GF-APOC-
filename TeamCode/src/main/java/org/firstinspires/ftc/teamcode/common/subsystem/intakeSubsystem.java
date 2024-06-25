package org.firstinspires.ftc.teamcode.common.subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotConstants;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JSubsystem;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

public class intakeSubsystem extends JSubsystem {
    robotHardware robot = robotHardware.getInstance();
    private double rollerVelocity;
    private boolean isOverCurrentLimit;
    private double rollerPower = 0;
    public boolean issueColorSensorCheck = false;
    private double lastCheckTime = 0;
    private boolean isLeftPixel = false;
    private boolean isRightPixel = false;
    public double v4BarAngle = Math.toRadians(0);
    private double transferFlapAngle = 0;
    private double latchPos = 0;
    public static double rollerDepth = 0.2;
    public int targetStackHeight = 1;
    public boolean isRechecked = true;

    intakeState state = intakeState.stationary;

    public enum intakeState{
        intake,
        outtake,
        stationary,
        hang
    }
    public intakeSubsystem() {
        issueColorSensorCheck = false;
    }
    public intakeSubsystem(double rollerDepth, int targetStackHeight) {
        setRollerDepth(rollerDepth);
        this.targetStackHeight = targetStackHeight;
    }


    @Override
    public void read(){
        rollerVelocity = robot.doubleSubscriber(Sensors.SensorType.INTAKE_VELOCITY);
        isOverCurrentLimit = robot.boolSubscriber(Sensors.SensorType.INTAKE_CURRENT);

        if(issueColorSensorCheck){
            lastCheckTime = robot.getTimeMs();
            isLeftPixel = (robot.leftColorSensor.getDistance(DistanceUnit.MM)<10);
            isRightPixel = (robot.rightColorSensor.getDistance(DistanceUnit.MM)<10);
            issueColorSensorCheck = false;
        }
    }
    @Override
    public void periodic() {
        switch(state){
            case intake:
                rollerPower = robotConstants.maxRollerPower;
                transferFlapAngle = 0;
                break;
            case stationary:
                rollerPower = 0;
                transferFlapAngle = 90;
                break;
            case outtake:
                rollerPower = -robotConstants.maxRollerPower;
                transferFlapAngle = 0;
                break;
            case hang:
                rollerPower = 0;
                transferFlapAngle = 0;
                break;
        }
        if((isOverCurrentLimit) || (rollerVelocity < robotConstants.velocityLimit)){
            issueColorSensorCheck = true;
            isRechecked = false;
        } else if((robot.getTimeMs()>lastCheckTime+1000 && !isRechecked)){
            issueColorSensorCheck = true;
            isRechecked = true;
        }
        if (isLeftPixel && isRightPixel)
            latchPos=robotConstants.latchClose;
        else
            latchPos=robotConstants.latchOpen;
        v4BarAngle = v4BarInverseKinematics(targetStackHeight);

    }
    @Override
    public void write() {
        robot.intakeRoller.setPower(rollerPower);
        robot.v4Bar.setAngle(v4BarAngle);
        robot.transferFlap.setAngle(transferFlapAngle);
        robot.latch.setPosition(latchPos);
    }
    @Override
    public void reset() {
        rollerPower = 0;
        state = intakeState.stationary;
    }

    public void updateState(@NotNull intakeState state) {
        this.state = state;
    }
    public boolean getLeftPixel(){
        return isLeftPixel;
    }
    public boolean getRightPixel(){
        return isRightPixel;
    }
    public void setRollerDepth(double mm){
        rollerDepth = MathUtils.mmToInches(mm);
    }
    private static double v4BarInverseKinematics(int stackHeight){
        return -(Math.PI/2-Math.acos((robotConstants.v4BarHeight+rollerDepth-CenterstageConstants.stackHeights[stackHeight-1]-robotConstants.rollerLength)/robotConstants.v4BarRadius));
    }
    public void setV4BarAngle(int stackHeight){
        targetStackHeight = stackHeight;
    }
    public intakeState getState(){
        return state;
    }

}