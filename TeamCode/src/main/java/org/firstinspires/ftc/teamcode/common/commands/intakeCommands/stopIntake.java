package org.firstinspires.ftc.teamcode.common.commands.intakeCommands;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.intakeSubsystem;
import com.arcrobotics.ftclib.command.InstantCommand;

public class stopIntake extends InstantCommand{
    public stopIntake(){
        super(() -> robotHardware.getInstance().intake.updateState(intakeSubsystem.intakeState.stationary));
    }
}

