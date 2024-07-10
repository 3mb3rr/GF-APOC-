package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;

@Autonomous
@Disabled
public class ServoTestFour extends LinearOpMode {
    public JServo v4Bar, transferFlap, leftPitch, rightPitch, pivot, roll, fingerLeft, fingerRight, droneServo;
    double pivotpos,pitchpos,dronepos,fourbarangle;
    @Override
    public void runOpMode() throws InterruptedException {
        transferFlap = new JServo(hardwareMap.get(Servo.class, "flapServo"));
        v4Bar = new JServo(hardwareMap.get(Servo.class, "fourBarServo"));
        fingerRight = new JServo(hardwareMap.get(Servo.class, "fingerServoRight"));
        fingerLeft = new JServo(hardwareMap.get(Servo.class, "fingerServoLeft"));
        roll = new JServo(hardwareMap.get(Servo.class, "wristServo"));
        pivot = new JServo(hardwareMap.get(Servo.class, "pivotServo"));
        leftPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoLeft"));
        rightPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoRight"));
        droneServo = new JServo(hardwareMap.get(Servo.class, "droneLaunchServo"));

        roll.setAngularRange(0,Math.toRadians(0),0.56,Math.toRadians(180));
        leftPitch.setAngularRange(0.5,Math.toRadians(0),0.16,Math.toRadians(-96));
        rightPitch.setAngularRange(0.5,Math.toRadians(0),0.16,Math.toRadians(-96));
        pivot.setAngularRange(0.53,0,0.87,Math.toRadians(90));

        transferFlap.setAngularRange(0,0,1,Math.toRadians(90));
        v4Bar.setAngularRange(1,Math.toRadians(80),0.34,Math.toRadians(0));

        transferFlap.setPosition(0.5);
        v4Bar.setAngle(0);
        fingerLeft.setPosition(0.5);
        fingerRight.setPosition(0.5);
        roll.setPosition(0);
        pivot.setPosition(0.5);
        leftPitch.setPosition(0.5);
        rightPitch.setPosition(0.5);
        droneServo.setPosition(0.5);

        pivotpos=0.5;
        pitchpos=0.5;
        dronepos=1;
        waitForStart();

        while (!isStopRequested()){

            if (gamepad1.dpad_left){
                fourbarangle++;
                v4Bar.setAngle(Math.toRadians(fourbarangle));
                sleep(100);
            }
            if (gamepad1.dpad_right){
                fourbarangle--;
                v4Bar.setAngle(Math.toRadians(fourbarangle));
                sleep(100);
            }
            if (gamepad1.y){
                pitchpos=pitchpos+0.01;
                rightPitch.setPosition(pitchpos);
                leftPitch.setPosition(pitchpos);
                sleep(100);
            }
            if (gamepad1.a){
                pitchpos=pitchpos-0.01;
                rightPitch.setPosition(pitchpos);
                leftPitch.setPosition(pitchpos);
                sleep(100);
            }

            if (gamepad1.x){
                dronepos=dronepos-0.01;
                droneServo.setPosition(dronepos);
                sleep(100);
            }
            if (gamepad1.b){
                dronepos=dronepos+0.01;
                droneServo.setPosition(dronepos);
                sleep(100);
            }

            telemetry.addData("pitch",pitchpos);
            telemetry.addData("pivbot", pivotpos);
            telemetry.addData("doponew",dronepos);
            telemetry.addData("4bar",fourbarangle);
            telemetry.update();
        }
    }
}
