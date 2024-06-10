package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.pathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

/**
 * This is the FollowerConstants class. It holds many constants and parameters for various parts of
 * the Follower. This is here to allow for easier tuning of Pedro Pathing, as well as concentrate
 * everything tunable for the Paths themselves in one place.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
@Config
public class robotConstants {

    // Slide constants
    public static PIDController slideController = new PIDController(0.02, 0, 0.005);
    // ticks for slide to place in the first row
    public static int slideFirstRowTicks = 400;
    // how many ticks should be added for slides to reach the next row
    public static int slideRowIncreaseTicks = 100;
    // how many ticks should be added if angle is changed from 0 to 30 degrees
    public static int slideAngleIncreaseTicks = 50;

    public static double slideFF = 0.1;
    public static double slideFFHang = -0.7;
    // Angle of slide with the ground
    public static double slideAngle = Math.toRadians(60);
    // pitch of arm at different functions
    public static double dropPitch = Math.toRadians(70);
    public static double waitPitch = Math.toRadians(-90);
    public static double transferPitch = Math.toRadians(-120);
    // values for finger servoes
    public static double releasePos = 0.1;
    public static double grabPos = 1;

    public static double droneLaunchPosition = 1;
    public static double droneWaitPosition = 0;


    //intake constants
    // limits to check if pixel is in intake
    public static double currentLimit = 0;
    public static double velocityLimit = 0;
    // roller power
    public static double maxRollerPower = 0.8;
    // height of pivot point of v4bar from ground
    public static double v4BarHeight = MathUtils.mmToInches(171.25);
    // length of v4bar
    public static double v4BarRadius = MathUtils.mmToInches(104);
    // length of one spoke of the rollers in intake
    public static double rollerLength = MathUtils.mmToInches(45);
    // localization constants
    public static double par0YTicks = 0.0; // y position of the first parallel encoder (in tick units)
    public static double par1YTicks = 1.0; // y position of the second parallel encoder (in tick units)
    public static double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
    public static double inPerTick = 0.00001;

    // Pathing constants
    // This section is for setting the actual drive vector for the front left wheel, if the robot
    // is facing a heading of 0 radians with the wheel centered at (0,0)
    private static double xMovement = 81.34056;
    private static double yMovement = 65.43028;
    private static double[] convertToPolar = Point.cartesianToPolar(xMovement, -yMovement);
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0],convertToPolar[1]));

    // Large translational PIDF coefficients
    public static CustomPIDFCoefficients largeTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.1,
            0,
            0,
            0);

    // Feed forward constant added on to the large translational PIDF
    public static double largeTranslationalPIDFFeedForward = 0.015;

    // Large translational Integral
    public static CustomPIDFCoefficients largeTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    // the limit at which the heading PIDF switches between the large and small translational PIDFs
    public static double translationalPIDFSwitch = 3;

    // Small translational PIDF coefficients
    public static CustomPIDFCoefficients smallTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.3,
            0,
            0.01,
            0);

    // Small translational Integral value
    public static CustomPIDFCoefficients smallTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    // Feed forward constant added on to the small translational PIDF
    public static double smallTranslationalPIDFFeedForward = 0.015;

    // Large heading error PIDF coefficients
    public static CustomPIDFCoefficients largeHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            1,
            0,
            0,
            0);

    // Feed forward constant added on to the large heading PIDF
    public static double largeHeadingPIDFFeedForward = 0.01;

    // the limit at which the heading PIDF switches between the large and small heading PIDFs
    public static double headingPIDFSwitch = Math.PI/20;

    // Small heading error PIDF coefficients
    public static CustomPIDFCoefficients smallHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            5,
            0,
            0.08,
            0);

    // Feed forward constant added on to the small heading PIDF
    public static double smallHeadingPIDFFeedForward = 0.01;

    // Large drive PIDF coefficients
    public static CustomPIDFCoefficients largeDrivePIDFCoefficients = new CustomPIDFCoefficients(
            0.025,
            0,
            0.00001,
            0);

    // Feed forward constant added on to the large drive PIDF
    public static double largeDrivePIDFFeedForward = 0.01;

    // the limit at which the heading PIDF switches between the large and small drive PIDFs
    public static double drivePIDFSwitch = 20;

    // Small drive PIDF coefficients
    public static CustomPIDFCoefficients smallDrivePIDFCoefficients = new CustomPIDFCoefficients(
            0.02,
            0,
            0.000005,
            0);

    // Feed forward constant added on to the small drive PIDF
    public static double smallDrivePIDFFeedForward = 0.01;

    // Mass of robot in kilograms
    public static double mass = 10.65942;

    // Centripetal force to power scaling
    public static double centripetalScaling = 0.0005;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    public static double forwardZeroPowerAcceleration = -34.62719;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    public static double lateralZeroPowerAcceleration = -78.15554;

    // A multiplier for the zero power acceleration to change the speed the robot decelerates at
    // the end of paths.
    // Increasing this will cause the robot to try to decelerate faster, at the risk of overshoots
    // or localization slippage.
    // Decreasing this will cause the deceleration at the end of the Path to be slower, making the
    // robot slower but reducing risk of end-of-path overshoots or localization slippage.
    // This can be set individually for each Path, but this is the default.
    public static double zeroPowerAccelerationMultiplier = 4;

    // When the robot is at the end of its current Path or PathChain and the velocity goes below
    // this value, then end the Path. This is in inches/second.
    // This can be custom set for each Path.
    public static double pathEndVelocityConstraint = 0.1;

    // When the robot is at the end of its current Path or PathChain and the translational error
    // goes below this value, then end the Path. This is in inches.
    // This can be custom set for each Path.
    public static double pathEndTranslationalConstraint = 0.1;

    // When the robot is at the end of its current Path or PathChain and the heading error goes
    // below this value, then end the Path. This is in radians.
    // This can be custom set for each Path.
    public static double pathEndHeadingConstraint = 0.007;

    // When the t-value of the closest point to the robot on the Path is greater than this value,
    // then the Path is considered at its end.
    // This can be custom set for each Path.
    public static double pathEndTValueConstraint = 0.995;

    // When the Path is considered at its end parametrically, then the Follower has this many
    // milliseconds to further correct by default.
    // This can be custom set for each Path.
    public static double pathEndTimeoutConstraint = 500;

    // This is how many steps the BezierCurve class uses to approximate the length of a BezierCurve.
    public static int APPROXIMATION_STEPS = 1000;

    // This is scales the translational error correction power when the Follower is holding a Point.
    public static double holdPointTranslationalScaling = 0.45;

    // This is scales the heading error correction power when the Follower is holding a Point.
    public static double holdPointHeadingScaling = 0.35;

    // This is the number of times the velocity is recorded for averaging when approximating a first
    // and second derivative for on the fly centripetal correction. The velocity is calculated using
    // half of this number of samples, and the acceleration uses all of this number of samples.
    public static int AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;

    // This is the number of steps the binary search for closest point uses. More steps is more
    // accuracy, and this increases at an exponential rate. However, more steps also does take more
    // time.
    public static int BEZIER_CURVE_BINARY_STEP_LIMIT = 10;
}
