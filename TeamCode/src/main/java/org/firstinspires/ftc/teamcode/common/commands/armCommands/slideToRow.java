package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

import com.arcrobotics.ftclib.command.InstantCommand;

public class slideToRow extends InstantCommand{
    public slideToRow(int row){
        super(() -> robotHardware.getInstance().deposit.setSlideTargetRow(row));
    }
}
