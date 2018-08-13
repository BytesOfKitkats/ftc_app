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

public class BoKAutoBlueFar extends BoKAutoCommon
{
    private static int TURN_RIGHT_DEGREES = -90;
    private static double DT_MOVE_TO_CRYPTO = 22; // Inches! Must come off the balancing stone
    private static int DISTANCE_TO_LEFT_COL_CM = 51; // CM
    private static int DISTANCE_TO_CENTER_COL_CM = 68;
    private static int DISTANCE_TO_RIGHT_COL_CM = 88;

    // Constructor
    public BoKAutoBlueFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {

    }
}
