package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.droneSubsystem;

public class droneLaunch extends InstantCommand {
    public droneLaunch(){
        super(() -> robotHardware.getInstance().drone.updateState(droneSubsystem.droneState.launch));
    }
}
