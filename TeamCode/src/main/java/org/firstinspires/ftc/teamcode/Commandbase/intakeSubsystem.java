package org.firstinspires.ftc.teamcode.Commandbase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

public class intakeSubsystem extends SubsystemBase {
    DcMotorEx intakeRoller;
    ServoEx fourbar;
    ServoEx transferFlap;
    ArrayList currentData = new ArrayList<double[]>();
    ArrayList velocityData = new ArrayList<double[]>();
    double timeSum = 0;
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
        timeSum = timeSum+time;
        currentData.add(current);
        velocityData.add(velocity);
        while(timeSum>1000000000){
            double lastTime = ((double[]) currentData.get(0))[0];
            timeSum=timeSum-lastTime;
            currentData.remove(0);
            velocityData.remove(0);

        }

    }

}
