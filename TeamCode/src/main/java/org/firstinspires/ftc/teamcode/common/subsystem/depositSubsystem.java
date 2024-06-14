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
import org.firstinspires.ftc.teamcode.common.util.wrappers.JActuator;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JSubsystem;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

public class depositSubsystem extends JSubsystem {
    robotHardware robot = robotHardware.getInstance();
    private int slideTargetPosition = 0;
    private int slideTargetRow = 0;
    armState ArmState = armState.wait;
    dropperState leftDropperState = dropperState.release;
    dropperState rightDropperState = dropperState.release;
    private double pitchTargetAngle;
    private double pivotTargetAngle;
    private double leftFingerPos, rightFingerPos;
    private double rollAngle = 0;
    private double targetRollAngle = 0;
    private double botVoltage = 12;

    public enum armState{
        wait,
        transfer,
        drop,
        rearrange
    }
    public enum dropperState{
        grab,
        release
    }
    public depositSubsystem() {
    }

    @Override
    public void read(){
        robot.leftSlide.read();
        robot.rightSlide.read();
        botVoltage = robot.doubleSubscriber(Sensors.SensorType.BATTERY);
    }
    @Override
    public void periodic() {
        // TODO: wait and transfer values not put
        switch(ArmState){
            case wait:
                pitchTargetAngle = robotConstants.waitPitch;
                pivotTargetAngle = robotConstants.pivotTransferAngle;
                targetRollAngle = 0;
                break;
            case transfer:
                pitchTargetAngle = robotConstants.transferPitch;
                pivotTargetAngle = robotConstants.pivotTransferAngle;
                targetRollAngle = 0;
                break;
            case drop:
                pitchTargetAngle = robotConstants.dropPitch;
                pivotTargetAngle = getPivotAngle(pitchTargetAngle);
                targetRollAngle = rollAngle;
                break;
            case rearrange:
                pitchTargetAngle = robotConstants.dropPitch;
                pivotTargetAngle = robotConstants.pivotRearrangeAngle;
                targetRollAngle = Math.toRadians(180);
                break;
        }
        switch(leftDropperState){
            case release:
                leftFingerPos = robotConstants.releasePos;
                break;
            case grab:
                leftFingerPos = robotConstants.grabPos;
                break;
        }
        switch(rightDropperState){
            case release:
                rightFingerPos = robotConstants.releasePos;
                break;
            case grab:
                rightFingerPos = robotConstants.grabPos;
                break;
        }
        slideTargetPosition = (slideTargetRow-1)*robotConstants.slideRowIncreaseTicks+robotConstants.slideFirstRowTicks;
        if(rollAngle != 0 && rollAngle != 180) slideTargetPosition+=robotConstants.slideAngleIncreaseTicks;
        robot.leftSlide.setVoltageSupplier(new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return botVoltage;
            }
        });
        robot.rightSlide.setVoltageSupplier(new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return botVoltage;
            }
        });
        robot.leftSlide.setTargetPosition(slideTargetPosition);
        robot.rightSlide.setTargetPosition(slideTargetPosition);
        robot.leftSlide.periodic();
        robot.rightSlide.periodic();


    }
    @Override
    public void write() {
        robot.leftSlide.write();
        robot.rightSlide.write();

        robot.leftPitch.setAngle(pitchTargetAngle);
        robot.rightPitch.setAngle(pitchTargetAngle);
        robot.pivot.setAngle(pivotTargetAngle);
        robot.fingerLeft.setPosition(leftFingerPos);
        robot.fingerRight.setPosition(rightFingerPos);
        robot.roll.setAngle(targetRollAngle);

    }
    @Override
    public void reset() {
        slideTargetPosition = 0;
        ArmState = armState.wait;
        leftDropperState = dropperState.release;
        rightDropperState = dropperState.release;

    }
    public void setTargetPosition(int position){
        slideTargetPosition = position;
    }
    public boolean isReached(){
        return (robot.leftSlide.hasReached() || robot.rightSlide.hasReached());
    }
    private double getPivotAngle(double pitchAngle){
        return(Math.toRadians(90)+robotConstants.slideAngle-Math.toRadians(60)-pitchAngle);
    }

    public void setRollAngle(double angle){
        rollAngle = angle;
    }
    public void updateArmState(@NotNull depositSubsystem.armState state) {
        this.ArmState = state;
    }
    public void updateDropperState(@NotNull depositSubsystem.dropperState state, String side) {
        if(side.equals("left"))this.leftDropperState = state;
        else this.rightDropperState = state;
    }
    public void setSlideTargetRow(int row){
        slideTargetRow = row;
    }
    public int getSlideTargetRow() {return slideTargetRow;}
    public void setFeedForward(double ff){
        robot.leftSlide.setFeedforward(JActuator.FeedforwardMode.CONSTANT, ff);
    }
    public dropperState getLeftDropperState(){
        return leftDropperState;
    }
    public dropperState getRightDropperState(){
        return rightDropperState;
    }
    public armState getArmState(){
        return ArmState;
    }

}
