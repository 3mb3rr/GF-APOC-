package org.firstinspires.ftc.teamcode.common.pathing.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotConstants;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.acmerobotics.roadrunner.Pose2d;


/**
 * This is the ThreeWheelLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses the three wheel odometry set up. The diagram below, which is taken from
 * Road Runner, shows a typical set up.
 *
 * The view is from the bottom of the robot looking upwards.
 *
 * left on robot is y pos
 *
 * front on robot is x pos
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |   left (y pos)
 *    |              |
 *    |              |
 *    \--------------/
 *      front (x pos)
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
@Config
public class FusionLocalizer extends Localizer {
    private final robotHardware robot = robotHardware.getInstance();
    public static class Params {
        public double par0YTicks = robotConstants.par0YTicks; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = robotConstants.par1YTicks; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = robotConstants.perpXTicks; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();



    public final double inPerTick;
    private Pose2d pose;
    private Pose2d startPose = new Pose2d(0, 0, 0);
    private Pose2d displacementPose = new Pose2d(0, 0, 0);
    private PoseVelocity2d currentVelocityRR;
    private Pose currentVelocity;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;

    /**
     * This creates a new ThreeWheelLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public FusionLocalizer(HardwareMap map) {
        this(new Pose());
    }
    public FusionLocalizer() {
        this(new Pose());
    }

    /**
     * This creates a new ThreeWheelLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param setStartPose the Pose to start from
     */
    public FusionLocalizer(Pose setStartPose) {

        inPerTick = robotConstants.inPerTick; //(Placeholder value)
        setStartPose(setStartPose);
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return new Pose(pose.position.x, pose.position.y, pose.heading.toDouble());
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return currentVelocity.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = new Pose2d(setStart.getX(), setStart.getY(), setStart.getHeading());
    }


    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        displacementPose = new Pose2d(setPose.getX()-startPose.position.x, setPose.getY()-startPose.position.y, setPose.getHeading()-startPose.heading.toDouble());
    }

    /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders. Then, the robot's global change in position is calculated
     * using the pose exponential method.
     */

    @Override
    public void update() {
        PositionVelocityPair par0PosVel = robot.encoderSubscriber(Sensors.SensorType.POD_PAR0);
        PositionVelocityPair par1PosVel = robot.encoderSubscriber(Sensors.SensorType.POD_PAR1);
        PositionVelocityPair perpPosVel = robot.encoderSubscriber(Sensors.SensorType.POD_PERP);



        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[]{
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[]{
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[]{
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        displacementPose = displacementPose.plus(twist.value());
        pose = new Pose2d(displacementPose.position.x+startPose.position.x, displacementPose.position.y+startPose.position.y, displacementPose.heading.toDouble()+startPose.heading.toDouble());
        currentVelocityRR = twist.velocity().value();
        currentVelocity = new Pose(currentVelocityRR.linearVel.x, currentVelocityRR.linearVel.y, currentVelocityRR.angVel);

    }



    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return pose.heading.toDouble();
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return 0;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return 0;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return 0;
    }
}
