package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

import com.arcrobotics.ftclib.command.InstantCommand;

public class slideToPosition extends InstantCommand{
    public slideToPosition(int row){
        super(() -> robotHardware.getInstance().deposit.setTargetPosition(row));
    }
}
