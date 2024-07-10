package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;

import java.util.List;

@Config
@Autonomous
@Disabled
public class ServoTestFive extends LinearOpMode {

    private Servo v1,v2,v3,v4,v5,v6;
    private DcMotorEx m1,m2,m3,m4;


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStarted() && !isStopRequested()) {

            v1 = hardwareMap.get(Servo.class, "v1");
            v2 = hardwareMap.get(Servo.class, "v2");
            v3 = hardwareMap.get(Servo.class, "v3");
            v4 = hardwareMap.get(Servo.class, "v4");
            v5 = hardwareMap.get(Servo.class, "v5");
            v6 = hardwareMap.get(Servo.class, "v6");

            m1 = hardwareMap.get(DcMotorEx.class,"m1");
            m2 = hardwareMap.get(DcMotorEx.class,"m2");
            m3 = hardwareMap.get(DcMotorEx.class,"m3");
            m4 = hardwareMap.get(DcMotorEx.class,"m4");
        }

        while (!isStopRequested()){
            m1.setPower(1);
            m2.setPower(1);
            m3.setPower(1);
            m4.setPower(1);

            v1.setPosition(1);
            v2.setPosition(1);
            v3.setPosition(1);
            v4.setPosition(1);
            v5.setPosition(1);
            v6.setPosition(1);
            sleep(1000);
            v1.setPosition(0.5);
            v2.setPosition(0.5);
            v3.setPosition(0.5);
            v4.setPosition(0.5);
            v5.setPosition(0.5);
            v6.setPosition(0.5);
            sleep(1000);
            v1.setPosition(0);
            v2.setPosition(0);
            v3.setPosition(0);
            v4.setPosition(0);
            v5.setPosition(0);
            v6.setPosition(0);
            sleep(1000);


            telemetry.update();
//
//            telemetry.addData("target pitch left",leftPitch.getPosition());
//            telemetry.addData("target pitch right",rightPitch.getPosition());
//            telemetry.addData("target pivot",pivot.getPosition());
        }
    }
}
