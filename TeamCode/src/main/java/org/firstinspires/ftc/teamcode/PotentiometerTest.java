package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by krish on 9/1/2018.
 */
@TeleOp(name="BoK Potentiometer", group="BoKTele")
public class PotentiometerTest extends LinearOpMode{
    BoKPotentiometer potentiometer;
    AnalogInput analogInput;
    Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        analogInput = hardwareMap.analogInput.get("pot");
        servo = hardwareMap.servo.get("s");
        potentiometer = new BoKPotentiometer(analogInput, 180);

        servo.setPosition(1);
        Log.v("BoK","potInit"+potentiometer.getValue());
        waitForStart();

        servo.setPosition(0);
        sleep(1000);
        Log.v("BoK","Servo reached? "+ potentiometer.reachedPos(0));
    }
}