package org.firstinspires.ftc.teamcode.common.commands.droneCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.droneSubsystem;

public class droneToWait extends InstantCommand {
    public droneToWait(){
        super(() -> robotHardware.getInstance().drone.updateState(droneSubsystem.droneState.wait));
    }
}