package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

@Config
public class LimitSwitch implements DigitalChannel{
    private final DigitalChannel TouchSensor;

    public LimitSwitch(DigitalChannel touchSensor){
        TouchSensor = touchSensor;
        TouchSensor.setMode(Mode.INPUT);
    }

    public boolean isPressed() {
        return !getState();
    }

    @Override
    public void setMode(DigitalChannel.Mode mode){
        TouchSensor.setMode(mode);
    }

    @Override
    public boolean getState() {
        return TouchSensor.getState();
    }
    @Override
    public void setState(boolean state) {
    }

    @Override
    public void setMode(DigitalChannelController.Mode mode) {
        TouchSensor.setMode(mode);
    }

    @Override
    public DigitalChannel.Mode getMode() {
        return TouchSensor.getMode();
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return "";
    }

    @Override
    public String getConnectionInfo() {
        return "";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}