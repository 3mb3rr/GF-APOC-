package org.firstinspires.ftc.teamcode.common.pathing.localization;


import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

public class AprilTagConstants {
    public static final double Y_OFFSET = MathUtils.mmToInches(215);
    public static final double X_OFFSET = 0;
    private static final Pose CAMERA_POSE= new Pose(X_OFFSET, Y_OFFSET, 0);

    public static final Pose BLUE_BACKDROP_POSITION = CenterstageConstants.BLUE_BACKDROP_POSITION;
    public static final Pose RED_BACKDROP_POSITION = CenterstageConstants.RED_BACKDROP_POSITION;

    public static Pose convertBlueBackdropPoseToGlobal(Pose pipeline) {
        Pose offset = new Pose();

        offset.setX(CAMERA_POSE.getX() * Math.cos(pipeline.getHeading()) - CAMERA_POSE.getY() * Math.sin(pipeline.getHeading()));
        offset.setY(CAMERA_POSE.getX() * Math.sin(pipeline.getHeading()) + CAMERA_POSE.getY() * Math.cos(pipeline.getHeading()));

        pipeline.add(offset);
        pipeline.subtract(BLUE_BACKDROP_POSITION);
        return pipeline;
    }

    public static Pose convertRedBackdropPoseToGlobal(Pose pipeline) {
        Pose offset = new Pose();

        offset.setX(CAMERA_POSE.getX() * Math.cos(pipeline.getHeading()) - CAMERA_POSE.getY() * Math.sin(pipeline.getHeading()));
        offset.setY(CAMERA_POSE.getX() * Math.sin(pipeline.getHeading()) + CAMERA_POSE.getY() * Math.cos(pipeline.getHeading()));

        pipeline.add(offset);
        pipeline.subtract(RED_BACKDROP_POSITION);
        return pipeline;
    }
}
