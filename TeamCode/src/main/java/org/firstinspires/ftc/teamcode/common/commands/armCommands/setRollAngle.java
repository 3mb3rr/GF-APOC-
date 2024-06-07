package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import com.arcrobotics.ftclib.command.InstantCommand;

public class setRollAngle extends InstantCommand{
    public setRollAngle(double Angle){
        super(() -> robotHardware.getInstance().deposit.setRollAngle(Angle));
    }
}

