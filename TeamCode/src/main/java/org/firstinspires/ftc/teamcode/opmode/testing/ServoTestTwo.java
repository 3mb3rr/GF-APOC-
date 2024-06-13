package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;

@Config
@Autonomous
public class ServoTestTwo extends LinearOpMode {
    public JServo v4Bar, transferFlap, leftPitch, rightPitch, pivot, roll, fingerLeft, fingerRight, droneServo;
    public static int pitch=0;
    public static int pivott=90;
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStarted() && !isStopRequested()) {
            telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            transferFlap = new JServo(hardwareMap.get(Servo.class, "flapServo"));
            v4Bar = new JServo(hardwareMap.get(Servo.class, "fourBarServo"));
            fingerRight = new JServo(hardwareMap.get(Servo.class, "fingerServoRight"));
            fingerLeft = new JServo(hardwareMap.get(Servo.class, "fingerServoLeft"));
            roll = new JServo(hardwareMap.get(Servo.class, "wristServo"));
            pivot = new JServo(hardwareMap.get(Servo.class, "pivotServo"));
            leftPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoLeft"));
            rightPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoRight"));
            droneServo = new JServo(hardwareMap.get(Servo.class, "droneServo"));

            roll.setAngularRange(0,Math.toRadians(0),0.56,Math.toRadians(180));
            leftPitch.setAngularRange(0.5,Math.toRadians(4),0.1,Math.toRadians(-107));
            rightPitch.setAngularRange(0.5,Math.toRadians(4),0.1,Math.toRadians(-107));
            pivot.setAngularRange(0.56,0,0.86,Math.toRadians(60));
            transferFlap.setAngularRange(0,0,1,Math.toRadians(90));
            v4Bar.setAngularRange(1,Math.toRadians(15),0.34,Math.toRadians(90));
        }

        while (!isStopRequested()){

            pivot.setAngle(Math.toRadians(pivott));
            leftPitch.setAngle(Math.toRadians(pitch));
            rightPitch.setAngle(Math.toRadians(pitch));


            telemetry.addData("pivot",pivott);
            telemetry.addData("pitch",pitch);


            telemetry.addData("target pitch left",leftPitch.getPosition());
            telemetry.addData("target pitch right",rightPitch.getPosition());
            telemetry.addData("target pivot",pivot.getPosition());

            telemetry.update();
        }
    }
}
