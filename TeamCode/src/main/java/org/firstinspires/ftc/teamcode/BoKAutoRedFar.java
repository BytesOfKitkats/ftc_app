package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    // Constructor
    public BoKAutoRedFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
        far = true;
    }

    @Override
    public void runSoftware()
    {
        int captureCounter = 0;
        boolean buttonPressed = false;
        while (opMode.opModeIsActive()) {
            if (opMode.gamepad1.a && !buttonPressed) {
                takePicture(String.format("VuImage_%d.png", captureCounter++));
            }
            buttonPressed = opMode.gamepad1.a;
        }
    }
}
