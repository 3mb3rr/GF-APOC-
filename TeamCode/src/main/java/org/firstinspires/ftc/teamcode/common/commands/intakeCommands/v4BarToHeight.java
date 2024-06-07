package org.firstinspires.ftc.teamcode.common.commands.intakeCommands;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

import com.arcrobotics.ftclib.command.InstantCommand;

public class v4BarToHeight extends InstantCommand{
    public v4BarToHeight(int stackHeight){
        super(() -> robotHardware.getInstance().intake.setV4BarAngle(stackHeight));
    }
}

