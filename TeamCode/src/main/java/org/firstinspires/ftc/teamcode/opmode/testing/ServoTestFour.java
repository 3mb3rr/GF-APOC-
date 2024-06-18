package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ServoTestFour extends LinearOpMode {
    Servo gayassServo1;
    Servo gayassServo2;
    Servo gayassServoWrist;
    @Override
    public void runOpMode() throws InterruptedException {
        gayassServo1=hardwareMap.get(Servo.class,"pitchServoLeft");
        gayassServo2=hardwareMap.get(Servo.class,"pitchServoRight");
       // gayassServoWrist=hardwareMap.get(Servo.class,"pivotServo");
        setGayAss(0.5);

        waitForStart();

        while (!isStopRequested()){
            if (gamepad1.x)
                setGayAss(1);
            if (gamepad1.y)
                setGayAss(0);
//            if (gamepad1.a)
//                gayassServoWrist.setPosition(0.5);
//            if (gamepad1.b) {
//                gayassServoWrist.setPosition(1);
//            }
//            if (gamepad1.x) {
//                gayassServoWrist.setPosition(0);
//            }
        }
    }
    public void setGayAss(double valorantplayers){
        gayassServo1.setPosition(valorantplayers);
        gayassServo2.setPosition(valorantplayers);
    }
}
