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
public class ServoTestTwo extends LinearOpMode {
    public JServo  leftPitch, rightPitch, pivot,fingL,fingR, latch,flap;
    public static int pitch=0;
    public static int pivott=90;
    public static double latchPos=0.5;
    public static int flapangle=90;
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStarted() && !isStopRequested()) {
            //telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            pivot = new JServo(hardwareMap.get(Servo.class, "pivotServo"));
            leftPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoLeft"));
            rightPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoRight"));
            fingL = new JServo(hardwareMap.get(Servo.class, "fingerServoLeft"));
            fingR = new JServo(hardwareMap.get(Servo.class, "fingerServoRight"));
            latch = new JServo(hardwareMap.get(Servo.class, "latchServo"));
            flap = new JServo(hardwareMap.get(Servo.class, "flapServo"));


            leftPitch.setAngularRange(0.5,Math.toRadians(0),0.16,Math.toRadians(-96));
            rightPitch.setAngularRange(0.5,Math.toRadians(0),0.16,Math.toRadians(-96));
            pivot.setAngularRange(0.53,0,0.87,Math.toRadians(90));
            flap.setAngularRange(0,0,1,Math.toRadians(90));

            leftPitch.setAngle(Math.toRadians(pitch));
            rightPitch.setAngle(Math.toRadians(pitch));
           //  pivot.setAngle(Math.toRadians(pivott));
            pivot.setPosition(0);
        }

        while (!isStopRequested()){
            if(gamepad1.a) {
                pitch++;
                leftPitch.setAngle(Math.toRadians(pitch));
                rightPitch.setAngle(Math.toRadians(pitch));
                sleep(100);
            }
            if (gamepad1.b) {
                pitch--;
                leftPitch.setAngle(Math.toRadians(pitch));
                rightPitch.setAngle(Math.toRadians(pitch));
                sleep(100);
            }
            if (gamepad1.right_bumper) {
                pivott++;
                pivot.setAngle(Math.toRadians(pivott));
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                pivott--;
                pivot.setAngle(Math.toRadians(pivott));
                sleep(100);
            }
            if (gamepad1.right_trigger>0.1) {
                fingL.setPosition(0.1);
                fingR.setPosition(0.1);
            }
            if (gamepad1.left_trigger>0.1){
                fingL.setPosition(0.95);
                fingR.setPosition(0.95);
            }
            if(gamepad1.x) {
                flapangle++;
                flap.setAngle(Math.toRadians(flapangle));
                sleep(100);
            }
            if (gamepad1.y) {
                flapangle--;
                flap.setAngle(Math.toRadians(flapangle));
                sleep(100);
            }
            if (gamepad1.dpad_right){
                latchPos+=0.01;
                latch.setPosition(latchPos);
                sleep(100);
            }
            if (gamepad1.dpad_left){
                latchPos-=0.01;
                latch.setPosition(latchPos);
                sleep(100);
            }
//            if (gamepad1.dpad_left){ pivot.setPosition(0);}
//            if (gamepad1.dpad_right) pivot.setPosition(1);
            telemetry.addData("pivot",pivott);
            telemetry.addData("pitch",pitch);
            telemetry.addData("latchpos",latchPos);
            telemetry.addData("flapos",flapangle);
            telemetry.update();
//
//            telemetry.addData("target pitch left",leftPitch.getPosition());
//            telemetry.addData("target pitch right",rightPitch.getPosition());
//            telemetry.addData("target pivot",pivot.getPosition());
        }
    }
}
