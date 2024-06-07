package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import com.arcrobotics.ftclib.command.InstantCommand;

public class armToTransferPosition extends InstantCommand{
    public armToTransferPosition(){
        super(() -> robotHardware.getInstance().deposit.updateArmState(depositSubsystem.armState.transfer));
    }
}
