package org.firstinspires.ftc.teamcode.common.pathing.examples;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.pathing.follower.Follower;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.followerSubsystem;

/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 */
@TeleOp(name = "Pedro Pathing TeleOp Enhancements", group = "Test")
public class TeleOpEnhancements extends OpMode {
    private final robotHardware robot = robotHardware.getInstance();

    GamepadEx pad1;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private Vector driveVector;
    private Vector headingVector;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.follower.setAuto(CenterstageConstants.IS_AUTO);
        robot.follower.read();
        robot.follower.periodic();
        robot.poseUpdater.periodic();
        robot.follower.write();

        pad1=new GamepadEx(gamepad1);

        leftFront = robot.leftFront;
        leftRear = robot.leftRear;
        rightRear = robot.rightRear;
        rightFront = robot.rightFront   ;

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveVector = new Vector();
        headingVector = new Vector();
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        robot.follower.read();

        driveVector.setOrthogonalComponents(-pad1.getLeftY(), -pad1.getLeftX());
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(robot.follower.getPose().getHeading());

        headingVector.setComponents(-pad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)+pad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), robot.follower.getPose().getHeading());

        robot.follower.setMovementVectors(robot.follower.getCentripetalForceCorrection(), headingVector, driveVector);

        robot.follower.periodic();
        robot.poseUpdater.periodic();
        robot.follower.write();
    }
}
