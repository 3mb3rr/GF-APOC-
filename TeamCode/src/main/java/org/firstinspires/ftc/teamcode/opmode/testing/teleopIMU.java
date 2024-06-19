package org.firstinspires.ftc.teamcode.opmode.testing;

//import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



/**
 * This file is an abuse to society, more digitized and more convientent doesn't mean shit. This file is animal.
 * There's absolutely no speed in this shit.
 *
 * This particular OpMode is the epitome of why dogshit is dogshit.
 * IMU or not this is SO SLOW that its disgusting.
 */
@TeleOp

public class teleopIMU extends LinearOpMode {
    BNO055IMU imu;
    Orientation angles;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    @Override
    public void runOpMode() throws InterruptedException {
        // IMU mapping

        setImu();






        // Chassis drive mapping
        telemetry.addData("Status", "Initialized");
        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront" );
        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront" );
        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack" );
        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack" );
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        while ((!isStopRequested()) && (!isStarted()))
        {

        }
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // main part of code: chassis driving
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double r = Math.hypot(gamepad1.left_stick_y,-gamepad1.left_stick_x);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4  - ((angles.firstAngle/180)*Math.PI);
            double rightX = (gamepad1.left_trigger-gamepad1.right_trigger)/2;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {


                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);

            }
            else{
                brake();

            }
        }
    }


    public void brake() {

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(100);


    }



    public void setImu(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }


}