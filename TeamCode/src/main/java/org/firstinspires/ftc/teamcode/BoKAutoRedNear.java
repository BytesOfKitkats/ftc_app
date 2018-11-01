package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedNear extends BoKAutoCommon {
    // Constructor
    public BoKAutoRedNear()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()

    {
        boolean buttonXPressed = false;
        int count =0;
        while(opMode.opModeIsActive()) {
            if(opMode.gamepad1.x && !buttonXPressed){
                takePicture("VuImage_"+count+".png");
                count++;
            }
            buttonXPressed = opMode.gamepad1.x;
        }
    }
}
