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
import java.util.function.DoubleSupplier;

public class depositSubsystem extends JSubsystem {
    robotHardware robot = robotHardware.getInstance();
    public int slideTargetPosition = 0;
    private int slideTargetRow = 0;
    armState pivotState;
    armState pitchState;
    dropperState leftDropperState;
    dropperState rightDropperState;
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
        rearrange,
        spikedrop
    }
    public enum dropperState{
        grab,
        release
    }
    public depositSubsystem() {
        if (CenterstageConstants.IS_AUTO){
            pivotState = armState.transfer;
            pitchState = armState.transfer;
            leftDropperState = dropperState.grab;
            rightDropperState = dropperState.grab;
        }
        else {
            pivotState = armState.wait;
            pitchState = armState.wait;
            leftDropperState = dropperState.release;
            rightDropperState = dropperState.release;
        }
    }

    @Override
    public void read(){
        robot.lift.read();
        botVoltage = robot.doubleSubscriber(Sensors.SensorType.BATTERY);

    }
    @Override
    public void periodic() {
        // TODO: wait and transfer values not put
        switch(pitchState){
            case wait:
                pitchTargetAngle = robotConstants.waitPitch;
                break;
            case transfer:
                pitchTargetAngle = robotConstants.transferPitch;
                break;
            case drop:
                pitchTargetAngle = robotConstants.dropPitch;
                break;
            case rearrange:
                pitchTargetAngle = robotConstants.rearrangePitch;
                break;
            case spikedrop:
                pitchTargetAngle = robotConstants.spikePitch;
                break;
        }
        switch (pivotState){
            case wait:
                pivotTargetAngle = robotConstants.pivotWaitAngle;
                targetRollAngle = 0;
                break;
            case transfer:
                pivotTargetAngle = robotConstants.pivotTransferAngle;
                targetRollAngle = 0;
                break;
            case drop:
                pivotTargetAngle = getPivotAngle(pitchTargetAngle);
                targetRollAngle = rollAngle;
                break;
            case rearrange:
                pivotTargetAngle = robotConstants.pivotRearrangeAngle;
                targetRollAngle = 0;
                break;
            case spikedrop:
                pivotTargetAngle = robotConstants.pivotSpikeAngle;
                targetRollAngle = 0;
                break;
        }
        switch(leftDropperState){
            case release:
                leftFingerPos = robotConstants.releasePos;
                break;
            case grab:
                leftFingerPos = robotConstants.grabPosL;
                break;
        }
        switch(rightDropperState){
            case release:
                rightFingerPos = robotConstants.releasePos;
                break;
            case grab:
                rightFingerPos = robotConstants.grabPosR;
                break;
        }
        if(slideTargetRow!=0){
            slideTargetPosition = (slideTargetRow-1)*robotConstants.slideRowIncreaseTicks+robotConstants.slideFirstRowTicks;}
        else{
            slideTargetPosition = 0;
        }
        if(rollAngle != 0 && rollAngle != Math.toRadians(180)) slideTargetPosition+=robotConstants.slideAngleIncreaseTicks;

        robot.lift.setTargetPosition(slideTargetPosition);
        robot.lift.periodic();


    }
    @Override
    public void write() {
        robot.lift.write();

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
        pitchState = armState.wait;
        pivotState = armState.wait;
        leftDropperState = dropperState.release;
        rightDropperState = dropperState.release;

    }
    public void setTargetPosition(int position){
        slideTargetPosition = position;
    }
    public boolean isReached(){
        return (robot.lift.hasReached());
    }
    private double getPivotAngle(double pitchAngle){
        return(Math.toRadians(90)+robotConstants.slideAngle-Math.toRadians(60)-pitchAngle);
    }

    public void setRollAngle(double angle){
        rollAngle = angle;
    }
    public double getRollAngle(){return rollAngle;}
    public void updatePitchState(@NotNull depositSubsystem.armState state) {
        this.pitchState = state;
    }
    public void updatePivotState(@NotNull depositSubsystem.armState state) {
        this.pivotState = state;
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
        robot.lift.setFeedforward(JActuator.FeedforwardMode.CONSTANT, ff);
    }
    public dropperState getLeftDropperState(){
        return leftDropperState;
    }
    public dropperState getRightDropperState(){
        return rightDropperState;
    }
    public armState getPivotState(){
        return pivotState;
    }
    public armState getPitchState(){
        return pitchState;
    }

}