package org.firstinspires.ftc.teamcode.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class jussLooptimeTest extends OpMode {
    private double currentTime=0;
    private double lastTime = 0.0;
    private double loopTime=0;
    public void init(){

    }
    public void loop(){
        currentTime=System.nanoTime();
        loopTime=currentTime - lastTime;
        lastTime = currentTime;

        telemetry.addData("looptime",loopTime);
    }
}
