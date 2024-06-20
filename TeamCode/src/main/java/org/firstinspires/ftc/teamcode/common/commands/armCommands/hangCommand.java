package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotConstants;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

import com.arcrobotics.ftclib.command.InstantCommand;

public class hangCommand extends InstantCommand{
    public hangCommand(){
        super(() -> robotHardware.getInstance().deposit.setFeedForward(robotConstants.slideFFHang));
    }
    public void hang(){
        robotHardware.getInstance().deposit.setFeedForward(robotConstants.slideFFHang);
        robotHardware.getInstance().deposit.setSlideTargetRow(0);
    }
}
