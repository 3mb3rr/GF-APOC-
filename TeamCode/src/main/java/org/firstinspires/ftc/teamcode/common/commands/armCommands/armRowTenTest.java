package org.firstinspires.ftc.teamcode.common.commands.armCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

public class armRowTenTest extends InstantCommand {
    public armRowTenTest(){
        super(() -> robotHardware.getInstance().deposit.setSlideTargetRow(8));
    }
}
