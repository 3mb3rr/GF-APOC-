package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;

@Config
@Autonomous
@Disabled
public class ServoTestSix extends LinearOpMode {
    public JServo  droneServo,droneHeight;
    public static int pitch=0;
    public static int pivott=90;
    public static double dronePos=0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStarted() && !isStopRequested()) {
            //telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            droneServo = new JServo(hardwareMap.get(Servo.class, "droneLaunchServo"));
            droneHeight = new JServo(hardwareMap.get(Servo.class, "droneHeightServo"));


            droneServo.setPosition(0);
        }

        while (!isStopRequested()){
            if(gamepad1.x && gamepad1.left_bumper) {
                droneServo.setPosition(1);
                sleep(100);
                droneServo.setPosition(0);
                sleep(100);
                droneServo.setPosition(1);
                sleep(100);
                droneServo.setPosition(0);
            }


            if (gamepad1.dpad_up){
                dronePos+=0.01;
                droneHeight.setPosition(dronePos);
                sleep(100);
            }
            if (gamepad1.dpad_down){
                dronePos-=0.01;
                droneHeight.setPosition(dronePos);
                sleep(100);
            }

            if (gamepad1.right_trigger>0.1) dronePos=0.83;
            if (gamepad1.left_trigger>0.1) dronePos=1;

            telemetry.addData("dronepos",dronePos);
            telemetry.update();
        }
    }
}
