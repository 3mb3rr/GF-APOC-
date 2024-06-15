package org.firstinspires.ftc.teamcode.common.subsystem;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JSubsystem;

public class slideSub extends JSubsystem {
    robotHardware robot = robotHardware.getInstance();
    int target = 0;
    @Override
    public void read(){
        robot.lift.read();
    }

    @Override
    public void periodic(){
        robot.lift.setTargetPosition(target);
        robot.lift.periodic();
    }

    @Override
    public void write(){
        robot.lift.write();
    }
    @Override
    public void reset(){}

    public void setTarget(int tar){
        target = tar;
    }
}
