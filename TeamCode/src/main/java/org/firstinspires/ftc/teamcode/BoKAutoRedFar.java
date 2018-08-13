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
    private static double TIMEOUT_DUMP = 1;
    private static double DT_MOVE_TO_CRYPTO = 25.5;//inches
    private static int DISTANCE_TO_RIGHT_COL_CM = 49;//cm
    private static int DISTANCE_TO_CENTER_COL_CM = 66;//cm
    private static int DISTANCE_TO_LEFT_COL_CM = 86;//cm

    // Constructor
    public BoKAutoRedFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {

    }
}
