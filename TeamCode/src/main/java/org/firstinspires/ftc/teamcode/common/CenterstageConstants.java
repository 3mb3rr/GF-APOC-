package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.common.pathing.localization.Pose;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.vision.Location;


public class CenterstageConstants {

    public static double[] stackHeights = {0.5, 1, 1.5, 2, 2.5};
    public static Pose redDrivetrainScoringPosition= new Pose(0, 0, 0);
    public static Pose blueDrivetrainScoringPosition= new Pose(0, 0, 0);

    public static Pose redStackIntakingPosition= new Pose(0, 0, 0);
    public static Pose blueStackIntakingPosition= new Pose(0, 0, 0);
    public static final Pose BLUE_BACKDROP_POSITION = new Pose(-35.5, 61.25, 0);
    public static final Pose RED_BACKDROP_POSITION = new Pose(35.5, 61.25, 0);
    public static Location SIDE = Location.FAR;
    /**
     * Match constants.
     */
    public static boolean IS_AUTO = true;
    public static Location ALLIANCE = Location.RED;
    public static Location RANDOMIZATION = Location.LEFT;
    public static Location PRELOAD = Location.LEFT;
    public static Location ROUTE = Location.STAGEDOOR;
    public static Path toSpikeMiddle = new Path(new BezierLine(new Point(0, 0,Point.CARTESIAN), new Point(21, 0,Point.CARTESIAN)));
//    toSpike.setConstantHeadingInterpolation(Math.toRadians(0));
    public static Path toSpikeLeft = new Path(new BezierCurve(new Point(0, 0,Point.CARTESIAN), new Point(21.8, -2.7,Point.CARTESIAN)));
//    toSpike.setConstantHeadingInterpolation(Math.toRadians(90));

    public static Path toSpikeRight = new Path(new BezierCurve(new Point(0, 0,Point.CARTESIAN), new Point(22.3, 2.9,Point.CARTESIAN)));
//    toSpike.setConstantHeadingInterpolation(Math.toRadians(-90));

//    public Path getPath()

}

