package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import com.arcrobotics.ftclib.command.InstantCommand;

public class setRollAngle extends InstantCommand{
    public setRollAngle(double angle){
        super(() -> robotHardware.getInstance().deposit.setRollAngle(angle));
    }
}

