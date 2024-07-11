package org.firstinspires.ftc.teamcode.opmode.testing;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.CenterstageConstants;
import org.firstinspires.ftc.teamcode.common.pathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.common.robot.Sensors;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;

@TeleOp
public class telemetryAll extends LinearOpMode {
    private final robotHardware robot=robotHardware.getInstance();
    private double currentTime=0;
    private double lastTime = 0.0;
    private double loopTime=0;

    private Vector driveVector;
    private Vector headingVector;

    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested() && !isStarted()){
            CommandScheduler.getInstance().reset();
            CenterstageConstants.IS_AUTO = false;
            robot.init(hardwareMap);
            robot.follower.setAuto(CenterstageConstants.IS_AUTO);
            robot.read();
            robot.periodic();
            robot.write();
            driveVector = new Vector();
            headingVector = new Vector();
            while (opModeInInit()) {
                telemetry.addLine("Robot Initialized.");
                telemetry.update();
            }
        }
        while (!isStopRequested()){
            robot.read();
            robot.periodic();
            robot.write();
            telemetry.addData("left dist",robot.leftColorSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("right dist",robot.rightColorSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("sslidep pos",robot.lift.getPosition());
            telemetry.addData("sldie limit switxch",robot.boolSubscriber(Sensors.SensorType.SLIDE_LIMIT));
            telemetry.addData("par0",robot.encoderSubscriber(Sensors.SensorType.POD_PAR0));
            telemetry.addData("par0",robot.encoderSubscriber(Sensors.SensorType.POD_PAR0));
            telemetry.addData("par1",robot.encoderSubscriber(Sensors.SensorType.POD_PAR1));
            telemetry.addData("perp",robot.encoderSubscriber(Sensors.SensorType.POD_PERP));
            telemetry.addData("usback",robot.doubleSubscriber(Sensors.SensorType.BACK_DISTANCE));
            telemetry.addData("usright",robot.doubleSubscriber(Sensors.SensorType.RIGHT_DISTANCE));
            telemetry.addData("usleft",robot.doubleSubscriber(Sensors.SensorType.LEFT_DISTANCE));

            currentTime=System.nanoTime();
            loopTime=currentTime - lastTime;
            lastTime = currentTime;

            telemetry.addData("looptime",loopTime);
            telemetry.update();
        }

    }
}
