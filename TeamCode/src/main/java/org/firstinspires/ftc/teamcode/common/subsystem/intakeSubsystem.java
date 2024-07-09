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
    public boolean LL_R, LL_G, LLF_R, LLF_G, LR_R, LR_G, LRF_R, LRF_G;

    intakeState state;
    LEDState ledState;


    public enum intakeState{
        intake,
        outtake,
        stationary,
        hang,
        intakeInAuto
    }

    public enum LEDState{
        left,
        right,
        none,
        both
    }
    public intakeSubsystem() {
        issueColorSensorCheck = false;
        if (CenterstageConstants.IS_AUTO)
            state= intakeState.intakeInAuto;
        else
            state=intakeState.stationary;
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

        if(((isOverCurrentLimit) || (rollerVelocity < robotConstants.velocityLimit)) && (state==intakeState.intake || state==intakeState.outtake)){
            issueColorSensorCheck = true;
            isRechecked = false;
        } else if((robot.getTimeMs()>lastCheckTime+2500 && !isRechecked)){
            issueColorSensorCheck = true;
            isRechecked = true;
        }
        if (state==intakeState.intake || state==intakeState.outtake) latchPos=robotConstants.latchOpen;
        else if (isLeftPixel && isRightPixel || state==intakeState.stationary) latchPos=robotConstants.latchClose;
        else latchPos=robotConstants.latchOpen;

        v4BarAngle = v4BarInverseKinematics(targetStackHeight);

        if (isLeftPixel && isRightPixel)
            ledState=LEDState.both;
        else if (isRightPixel)
            ledState=LEDState.right;
        else if (isLeftPixel)
            ledState=LEDState.left;
        else
            ledState=LEDState.none;


        switch(state){
            case intake:
                rollerPower = robotConstants.maxRollerPower;
                transferFlapAngle = robotConstants.flapDown;
                v4BarAngle = v4BarInverseKinematics(4);
                break;
            case stationary:
                rollerPower = 0;
                transferFlapAngle = robotConstants.flapUp;
                if (!CenterstageConstants.IS_AUTO) v4BarAngle= Math.toRadians(75);
                break;
            case outtake:
                rollerPower = -robotConstants.maxRollerPower;
                transferFlapAngle = robotConstants.flapDown;
                break;
            case hang:
                rollerPower = 0;
                transferFlapAngle = robotConstants.flapDown;
                v4BarAngle = Math.toRadians(75);
                break;
            case intakeInAuto:
                transferFlapAngle = robotConstants.flapUp;
                v4BarAngle = Math.toRadians(75);
                latchPos = robotConstants.latchClose;
                break;
        }

        //true false inverted becasue false lights it up and true turns it off
        switch(ledState){
            case left:
                LL_G=true;
                LLF_G=true;
                LLF_R=false;
                LL_R=false;

                LR_G=false;
                LRF_G=false;
                LRF_R=true;
                LR_R=true;
                break;
            case right:
                LR_G=true;
                LRF_G=true;
                LRF_R=false;
                LR_R=false;

                LL_G=false;
                LLF_G=false;
                LLF_R=true;
                LL_R=true;
                break;
            case none:
                LR_G=false;
                LRF_G=false;
                LRF_R=true;
                LR_R=true;

                LL_G=false;
                LLF_G=false;
                LLF_R=true;
                LL_R=true;
                break;
            case both:
                LR_G=true;
                LRF_G=true;
                LRF_R=false;
                LR_R=false;

                LL_G=true;
                LLF_G=true;
                LLF_R=false;
                LL_R=false;
                break;
        }

    }
    @Override
    public void write() {
        robot.intakeRoller.setPower(rollerPower);
        robot.v4Bar.setAngle(v4BarAngle);
        robot.transferFlap.setAngle(transferFlapAngle);
        robot.latch.setPosition(latchPos);

//        robot.LEDLeftRed.enable(LL_R);
//        robot.LEDLeftFrontRed.enable(LLF_R);
//        robot.LEDRightFrontRed.enable(LRF_R);
//        robot.LEDRightRed.enable(LR_R);
//        robot.LEDLeftGreen.enable(LL_G);
//        robot.LEDLeftFrontGreen.enable(LLF_G);
//        robot.LEDRightFrontGreen.enable(LRF_G);
//        robot.LEDRightGreen.enable(LR_G);
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