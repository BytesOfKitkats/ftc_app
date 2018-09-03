package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by krish on 8/15/2018.
 */

public class BoKPotentiometer
{
    AnalogInput potentiometer;
    double range, maxValue;

    public BoKPotentiometer(AnalogInput pot, double servoRange)
    {
        potentiometer = pot;
        range = servoRange;
        maxValue = range/270;
    }

    boolean reachedPos(double servoPos)
    {
        double expectedPotValue = -maxValue*(servoPos-1);
        //expectedPotValue = expectedPotValue*expectedPotValue*0.00035+0.005*expectedPotValue;
        boolean reached =  (Math.abs(expectedPotValue-getValue()) <= 0.02);
        Log.v("BoK", "Pot pos " + getValue() + " expValue " + expectedPotValue);
        return  reached;
    }

    double getValue(){
        Log.v("BoK", "PotVoltage" + potentiometer.getVoltage());
        return  potentiometer.getVoltage()/3.3;
    }
}
