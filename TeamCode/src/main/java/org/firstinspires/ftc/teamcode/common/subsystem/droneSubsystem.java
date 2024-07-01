package org.firstinspires.ftc.teamcode.common.subsystem;


import org.firstinspires.ftc.teamcode.common.robot.robotConstants;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JSubsystem;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

public class droneSubsystem extends JSubsystem {
    robotHardware robot = robotHardware.getInstance();
    private double droneServoPosition = 0;
    public double droneHeightPosition = 0;
    droneState state = droneState.wait;
    public enum droneState{
        wait,
        launch,
        up
    }
    public droneSubsystem() {
    }

    @Override
    public void read(){
    }
    @Override
    public void periodic() {
        switch(state){
            case wait:
                droneServoPosition = robotConstants.droneWaitPosition;
                droneHeightPosition = robotConstants.droneHeightWaitPos;
                break;
            case launch:
                droneServoPosition = robotConstants.droneLaunchPosition;
                droneHeightPosition = robotConstants.droneHeightUpPos;
                break;
            case up:
                droneServoPosition = robotConstants.droneWaitPosition;
                droneHeightPosition = robotConstants.droneHeightUpPos;
                break;
        }
    }
    @Override
    public void write() {
        if(robot.droneServo.getPosition() != droneServoPosition) robot.droneServo.setPosition(droneServoPosition);
        if(robot.droneHeight.getPosition() != droneHeightPosition) robot.droneHeight.setPosition(droneHeightPosition);
    }


    @Override
    public void reset() {
        state = droneState.wait;

    }
    public void updateState(@NotNull droneSubsystem.droneState state) {
        this.state = state;
    }

}