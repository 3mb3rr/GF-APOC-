package org.firstinspires.ftc.teamcode.common.subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;

public class intakeSubsystem extends SubsystemBase {
    DcMotorEx intakeRoller;
    ServoEx fourbar;
    ServoEx transferFlap;
    ArrayList<double[]> currentData = new ArrayList<>();
    ArrayList<double[]> velocityData = new ArrayList<>();
    double avgCurrent = 0;
    double avgVelocity = 0;

    double startTime = 0;
    public intakeSubsystem(final HardwareMap hMap) {
        intakeRoller = hMap.get(DcMotorEx.class, "intakeRoller");
        fourbar = hMap.get(ServoEx.class, "fourbar");
        transferFlap = hMap.get(ServoEx.class, "transferFlap");
    }
    public void startTimer(){
        startTime = System.nanoTime();
    }
    public double getTime(){
        return (System.nanoTime()-startTime);
    }
    @Override
    public void periodic() {
        double time = getTime();
        double[] current = {time, intakeRoller.getCurrent(CurrentUnit.AMPS)};
        double[] velocity = {time, intakeRoller.getVelocity()};
        currentData.add(current);
        velocityData.add(velocity);
        for(int i =0; i< currentData.size(); i++){
            if(time-(currentData.get(i)[0])>1000000000) { currentData.remove(i); velocityData.remove(i); i--; }
            else { avgCurrent+=(currentData.get(i)[1]); avgVelocity+=(velocityData.get(i)[1]); }
        }
        avgCurrent = avgCurrent/currentData.size();
        avgVelocity = avgVelocity/velocityData.size();


    }

}
