package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;

@Config
public class AnalogDistanceSensor implements AnalogSensor {
    public static double INPUT_RANGE = 3.267;
    // in cm
    public static double SENSOR_MAX = 500;
    private final AnalogInput UltrasonicSensor;

    public AnalogDistanceSensor(AnalogInput sensor){
        UltrasonicSensor = sensor;
    }

    public void setSensorMax(double max) {
        SENSOR_MAX = max;
    }

    public double getDistance() {
        return SENSOR_MAX*(readRawVoltage()/INPUT_RANGE);
    }

    @Override
    public double readRawVoltage() {
        return UltrasonicSensor.getVoltage();
    }
}