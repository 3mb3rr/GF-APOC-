package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.robot.robotConstants;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JActuator;
import org.firstinspires.ftc.teamcode.common.util.wrappers.JServo;


@Config
@Autonomous
public class acutatorTest extends OpMode {
    private PIDController controller;
    private double pos=0.5;
    public static double p=0.02, i=0,d=0.000475;
    public static double f=0.1;
    public static int target=0;
    private DcMotorEx slideLeft;
    private DcMotorEx slideRight;
    private JActuator lift;

    public void init(){
        controller = new PIDController(p, i, d);
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        slideRight.setDirection(DcMotorEx.Direction.REVERSE);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift = new JActuator(() -> slideLeft.getCurrentPosition(), slideLeft, slideRight);
        lift.setPIDController(new PIDController(0.02, 0, 0.000475));
        lift.setFeedforward(JActuator.FeedforwardMode.CONSTANT, f);
        lift.setErrorTolerance(5);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("initilzed");

    }

    public void loop(){
        lift.read();
        lift.setTargetPosition(target);
        lift.periodic();
        lift.write();

        telemetry.addData("slide pos left", lift.getPosition());
        telemetry.addData("motor power", lift.getPower());
        telemetry.addData("target",target);

    }
}
