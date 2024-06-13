package org.firstinspires.ftc.teamcode;

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
    public JServo  leftPitch, rightPitch, pivot;
    public static int pitch=0;
    public static int pivott=90;
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStarted() && !isStopRequested()) {
            //telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            pivot = new JServo(hardwareMap.get(Servo.class, "pivotServo"));
            leftPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoLeft"));
            rightPitch = new JServo(hardwareMap.get(Servo.class, "pitchServoRight"));


            leftPitch.setAngularRange(0.5,Math.toRadians(4),0.1,Math.toRadians(-107));
            rightPitch.setAngularRange(0.5,Math.toRadians(4),0.1,Math.toRadians(-107));
            pivot.setAngularRange(0.56,0,0.86,Math.toRadians(60));

            leftPitch.setAngle(Math.toRadians(pitch));
            rightPitch.setAngle(Math.toRadians(pitch));
            pivot.setAngle(Math.toRadians(pivott));
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
            telemetry.addData("pivot",pivott);
            telemetry.addData("pitch",pitch);
            telemetry.update();
//
//            telemetry.addData("target pitch left",leftPitch.getPosition());
//            telemetry.addData("target pitch right",rightPitch.getPosition());
//            telemetry.addData("target pivot",pivot.getPosition());
        }
    }
}
