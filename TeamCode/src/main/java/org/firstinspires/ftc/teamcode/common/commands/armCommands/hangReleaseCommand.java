package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotConstants;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JActuator;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDController;

public class hangReleaseCommand extends InstantCommand{
    public hangReleaseCommand(){
        super(() -> hangRelease());
    }
    public static void hangRelease(){
        robotHardware.getInstance().lift.setFeedforward(JActuator.FeedforwardMode.CONSTANT,0.1);
        robotHardware.getInstance().lift.setPIDController(new PIDController(0.0005, 0, 0.000485));
        robotHardware.getInstance().deposit.setSlideTargetRow(4);
    }
}