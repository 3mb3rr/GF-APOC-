package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import com.arcrobotics.ftclib.command.InstantCommand;

public class pivotToSpikePosition extends InstantCommand{
    public pivotToSpikePosition(){
        super(() -> robotHardware.getInstance().deposit.updatePivotState(depositSubsystem.armState.spikedrop));
    }
}
