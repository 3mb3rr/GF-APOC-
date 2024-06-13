package org.firstinspires.ftc.teamcode.common.commands;

import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

import com.arcrobotics.ftclib.command.InstantCommand;

public class followPath extends InstantCommand{
    public followPath(Path path){
        super(() -> robotHardware.getInstance().follower.followPath(path, true));
    }
}
