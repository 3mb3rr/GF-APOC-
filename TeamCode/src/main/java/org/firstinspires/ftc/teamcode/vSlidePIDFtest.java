package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous
public class vSlidePIDFtest extends OpMode {
    private PIDController controller;
    private double pos=0.5;
    public static double p=0.02, i=0,d=0.0007;
    public static double f=0.1;
    public static int target=0;
    private DcMotorEx slideLeft;
    private DcMotorEx slideRight;
    private DcMotorEx intakeMotor;
    private Servo pitchServoLeft;
    private Servo pitchServoRight;
    private Servo pivotServo;
    private Servo wristServo;
    private Servo fingerServoLeft;
    private Servo fingerServoRight;
    private Servo fourbar;

    private TouchSensor slideCloseLeft;
    private TouchSensor slideCloseRight;
    private int slidePosLeft;
    private int slidePosRight;
    public ElapsedTime time;
    int j;
    public void init(){
        controller = new PIDController(p, i, d);
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        pitchServoLeft = hardwareMap.get(Servo.class, "pitchServoLeft");
        pitchServoRight = hardwareMap.get(Servo.class, "pitchServoRight");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        fingerServoLeft = hardwareMap.get(Servo.class, "fingerServoLeft");
        fingerServoRight = hardwareMap.get(Servo.class, "fingerServoRight");

        fourbar = hardwareMap.get(Servo.class, "fourBarServo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        slideCloseLeft = hardwareMap.touchSensor.get("slideCloseLeft");
        slideCloseRight = hardwareMap.touchSensor.get("slideCloseRight");
        slidePosLeft = 0;
        slidePosRight = 0;
        slideRight.setDirection(DcMotorEx.Direction.REVERSE);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("initilzed");
//        pitchServoLeft.setPosition(pos);
//        pitchServoRight.setPosition(pos);
        time = new ElapsedTime();
    }

    public void loop(){
        if (target==0 || target==500){
            time.reset();
            j=0;
        }

        slidePosLeft= slideLeft.getCurrentPosition();
        slidePosRight= slideRight.getCurrentPosition();

        controller.setPID(p,i,d);
        double pid=controller.calculate((slidePosLeft+slidePosRight)/2, target);
        double power = (pid +f);
        slideLeft.setPower(power);
        slideRight.setPower(power);

        telemetry.addData("slide pos left", slideLeft.getCurrentPosition());
        telemetry.addData("slide pos right",slideRight.getCurrentPosition());
        telemetry.addData("motor power", power);
        telemetry.addData("target",target);

        if (slideLeft.getVelocity()<1 && slidePosLeft>=1995) {
            if (j != 1) {
                double timetaken = time.seconds();
                telemetry.addData("the time taken", timetaken);
                j = 1;
            }
        }

        if (slideCloseLeft.isPressed() || slideCloseRight.isPressed()){
            slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


//        if (gamepad1.left_bumper) {
//            pos=pos-0.01;
//            pitchServoLeft.setPosition(pos);
//            pitchServoRight.setPosition(pos);
//        }
//
//        if (gamepad1.right_bumper) {
//            pos=pos+0.01;
//            pitchServoLeft.setPosition(pos);
//            pitchServoRight.setPosition(pos);
//        }

        telemetry.addData("position of pitch",pos);
    }
}
