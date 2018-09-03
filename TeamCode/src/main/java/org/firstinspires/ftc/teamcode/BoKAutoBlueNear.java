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

public class BoKAutoBlueNear extends BoKAutoCommon
{
    private static final double TIMEOUT_LEFT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_RIGHT = 6;

    private static final double DISTANCE_TO_LEFT_COL = 27; // inches!!
    private static final double DISTANCE_TO_CENTER_COL = 34;
    private static final double DISTANCE_TO_RIGHT_COL = 42;

    // Constructor
    public BoKAutoBlueNear()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        gyroTurn(0.25,0,-150,2,false,false,15);
    }
}
