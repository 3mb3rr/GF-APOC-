package org.firstinspires.ftc.teamcode.common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

@Config
public class LEDIndicator implements DigitalChannel{
    private final DigitalChannel Indicator;

    public LEDIndicator(DigitalChannel indicator){
        Indicator = indicator;
        Indicator.setMode(Mode.OUTPUT);
    }

    @Override
    public void setMode(DigitalChannel.Mode mode){
        Indicator.setMode(mode);
    }

    @Override
    public boolean getState() {
        return Indicator.getState();
    }

    @Override
    public void setState(boolean state) {
        Indicator.setState(state);
    }

    @Override
    public void setMode(DigitalChannelController.Mode mode) {
        Indicator.setMode(mode);
    }

    @Override
    public DigitalChannel.Mode getMode() {
        return Indicator.getMode();
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