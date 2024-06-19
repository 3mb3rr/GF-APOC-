package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;

@Autonomous
public class ServoTestFour extends LinearOpMode {
    public JServo v4Bar, transferFlap, leftPitch, rightPitch, pivot, roll, fingerLeft, fingerRight, droneServo;
    double pivotpos,pitchpos;
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
        droneServo = new JServo(hardwareMap.get(Servo.class, "droneServo"));

        transferFlap.setPosition(0.5);
        v4Bar.setPosition(0.5);
        fingerLeft.setPosition(0.5);
        fingerRight.setPosition(0.5);
        roll.setPosition(0);
        pivot.setPosition(0.5);
        leftPitch.setPosition(0.5);
        rightPitch.setPosition(0.5);
        droneServo.setPosition(0.5);

        pivotpos=0.5;
        pitchpos=0.5;
        waitForStart();

        while (!isStopRequested()){

            if (gamepad1.dpad_left){
                pivotpos=pivotpos-0.01;
                pivot.setPosition(pivotpos);
                sleep(100);
            }
            if (gamepad1.dpad_right){
                pivotpos=pivotpos+0.01;
                pivot.setPosition(pivotpos);
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

            telemetry.addData("pitch",pitchpos);
            telemetry.addData("pivbot", pivotpos);
            telemetry.update();
        }
    }
}
