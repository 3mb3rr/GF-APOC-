package org.firstinspires.ftc.teamcode.common.commands;

import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

import com.arcrobotics.ftclib.command.InstantCommand;

public class interruptFollower extends InstantCommand{
    public interruptFollower(){
        super(() -> robotHardware.getInstance().follower.breakFollowing());
    }
}
