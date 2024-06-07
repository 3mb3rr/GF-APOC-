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
    armState ArmState = armState.wait;
    dropperState leftDropperState = dropperState.release;
    dropperState rightDropperState = dropperState.release;
    private double pitchTargetAngle;
    private double pivotTargetAngle;
    private double leftFingerPos, rightFingerPos;
    private double[] rollAngles = {0, Math.toRadians(30), Math.toRadians(150), Math.toRadians(180), Math.toRadians(210), Math.toRadians(270)};
    private int rollIndex = 0;
    private double rollAngle = 0;

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
    }
    @Override
    public void periodic() {
        robot.leftSlide.setTargetPosition(slideTargetPosition);
        robot.leftSlide.setTargetPosition(slideTargetPosition);
        // TODO: wait and transfer values not put
        switch(ArmState){
            case wait:
                pitchTargetAngle = 0;
                pivotTargetAngle = getPivotAngle(pitchTargetAngle);
            case transfer:
                pitchTargetAngle = 0;
                pivotTargetAngle = getPivotAngle(pitchTargetAngle);
            case drop:
                pitchTargetAngle = 70;
                pivotTargetAngle = getPivotAngle(pitchTargetAngle);
            case rearrange:
                pitchTargetAngle = 70;
                pivotTargetAngle = 90;
        }
        switch(leftDropperState){
            case release:
                leftFingerPos = robotConstants.releasePos;
            case grab:
                leftFingerPos = robotConstants.grabPos;
        }
        switch(rightDropperState){
            case release:
                rightFingerPos = robotConstants.releasePos;
            case grab:
                rightFingerPos = robotConstants.grabPos;
        }

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
        if(rollIndex==-1)robot.roll.setAngle(rollAngle);
        else robot.roll.setAngle(rollAngles[rollIndex]);

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
    public void setRollIndex(int index){
        rollIndex = index;
    }
    public int getRollIndex(){ return rollIndex;}
    public void setRollAngle(double angle){
        rollIndex = -1;
        rollAngle = angle;
    }
    public void updateArmState(@NotNull depositSubsystem.armState state) {
        this.ArmState = state;
    }
    public void updateDropperState(@NotNull depositSubsystem.dropperState state, String side) {
        if(side.equals("left"))this.leftDropperState = state;
        else this.rightDropperState = state;
    }

}
