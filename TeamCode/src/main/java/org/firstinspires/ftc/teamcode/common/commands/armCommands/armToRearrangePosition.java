package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import com.arcrobotics.ftclib.command.InstantCommand;

public class armToRearrangePosition extends InstantCommand{
    public armToRearrangePosition(){
        super(() -> robotHardware.getInstance().deposit.updateArmState(depositSubsystem.armState.rearrange));
    }
}
