package org.firstinspires.ftc.teamcode.opmode.testing;

import com.outoftheboxrobotics.photoncore.hardware.PhotonLynxVoltageSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.outoftheboxrobotics.photoncore.Photon;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Photon
@Autonomous
public class LoopTimeWithColorSensor extends LinearOpMode {
    private RevColorSensorV3 csLeft;
    private RevColorSensorV3 csRight;
    private PhotonLynxVoltageSensor sensor;
    private double currentTime=0;
    private double lastTime = 0.0;
    private double loopTime=0;
    private ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested() && !isStarted()) {
            csLeft = hardwareMap.get(RevColorSensorV3.class, "CSLeft");
            csRight = hardwareMap.get(RevColorSensorV3.class, "CSRight");
            sensor = hardwareMap.getAll(PhotonLynxVoltageSensor.class).iterator().next();
        }

        while (!isStopRequested()){
            telemetry.addData("left dist",csLeft.getDistance(DistanceUnit.MM));
            telemetry.addData("right dist",csRight.getDistance(DistanceUnit.MM));

            currentTime=System.nanoTime();
            loopTime=currentTime - lastTime;
            lastTime = currentTime;

            telemetry.addData("looptime",loopTime);
            double voltage = sensor.getCachedVoltage();
            telemetry.addData("batery",voltage);
            telemetry.update();
        }
    }
}
