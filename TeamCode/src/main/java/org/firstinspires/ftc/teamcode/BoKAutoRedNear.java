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

public class BoKAutoRedNear extends BoKAutoCommon {
    private static final double TIMEOUT_RIGHT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_LEFT = 6;
    
    private static final double DISTANCE_TO_RIGHT_COL = 26; // inches!!
    private static final double DISTANCE_TO_CENTER_COL = 26;
    private static final double DISTANCE_TO_LEFT_COL = 33;

    // Constructor
    public BoKAutoRedNear()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {

    }
}
