package org.firstinspires.ftc.teamcode.common.commands.droneCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.droneSubsystem;

public class droneToHeightUp extends InstantCommand {
    public droneToHeightUp(){
        super(() -> robotHardware.getInstance().drone.updateState(droneSubsystem.droneState.up));
    }
}