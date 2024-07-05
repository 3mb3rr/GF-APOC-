package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class ServoTest extends LinearOpMode {
    private Servo fourbar,wrist;
    private DcMotorEx intakeMotor;
    private double pos=0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested() && !isStarted()) {
            fourbar = hardwareMap.get(Servo.class, "fourBarServo");
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            wrist = hardwareMap.get(Servo.class,"wristServo");
            pos=0.5;
            fourbar.setPosition(pos);
            wrist.setPosition(0);
        }

        while (!isStopRequested()){

            if (gamepad1.dpad_left){
                pos=pos-0.01;
                fourbar.setPosition(pos);
                sleep(100);
            }
            if (gamepad1.dpad_right){
                pos=pos+0.01;
                fourbar.setPosition(pos);
                sleep(100);
            }


            if (gamepad1.a)
                intakeMotor.setPower(0.8);
            if (gamepad1.b)
                intakeMotor.setPower(-0.8);
            if (gamepad1.x)
                intakeMotor.setPower(0);


            telemetry.addData("4barpos",pos);
            telemetry.update();
        }
    }
}
