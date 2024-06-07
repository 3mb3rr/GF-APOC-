package org.firstinspires.ftc.teamcode.common.commands.dropperCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

public class releaseLeftPixel extends InstantCommand{
    public releaseLeftPixel(){
        super(() -> robotHardware.getInstance().deposit.updateDropperState(depositSubsystem.dropperState.release, "left"));
    }
}