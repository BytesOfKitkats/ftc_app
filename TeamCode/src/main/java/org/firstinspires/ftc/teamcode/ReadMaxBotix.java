package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by shiv on 3/31/2018.
 */
@TeleOp(name="BOK MaxBotix", group="BoKTele")
public class ReadMaxBotix extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput mb1240;
        mb1240 = hardwareMap.analogInput.get("mbs");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("AI: ", mb1240.getVoltage() / 0.00189);
            telemetry.update();
            //Log.v("BOK: ",  "dist " + mb1240.getVoltage() / 0.00189);
        }
    }
}
