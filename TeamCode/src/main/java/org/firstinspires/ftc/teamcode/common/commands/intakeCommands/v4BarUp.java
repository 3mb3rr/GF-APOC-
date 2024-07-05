package org.firstinspires.ftc.teamcode.common.commands.intakeCommands;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

import com.arcrobotics.ftclib.command.InstantCommand;

public class v4BarUp extends InstantCommand{
    public v4BarUp(){
        super(() -> robotHardware.getInstance().v4Bar.setAngle(75));
    }
}

