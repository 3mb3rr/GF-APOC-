package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.pathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.droneSubsystem;

public class AprilTagPose extends InstantCommand {
    public AprilTagPose(){
        super(() -> robotHardware.getInstance().poseUpdater.setPose(new Pose(
                robotHardware.getInstance().getAprilTagPosition().getY(),
                robotHardware.getInstance().getAprilTagPosition().getX(),
                robotHardware.getInstance().getAngle())
        ));
    }

}