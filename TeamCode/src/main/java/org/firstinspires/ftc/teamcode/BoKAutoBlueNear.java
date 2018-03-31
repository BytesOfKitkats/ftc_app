package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueNear extends BoKAutoCommon
{
    private static final double TIMEOUT_LEFT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_RIGHT = 6;

    private static final double DISTANCE_TO_LEFT_COL = 26; // inches!!
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
        // NOTE: Move backwards towards crypto

        // Detect Vuforia image and flick the jewel
        detectVuforiaImgAndFlick(WAIT_FOR_JEWEL_FLICKER_MS);

        // Move backward out of balancing stone
        // Distance and timeout to the cryptobox depends on column number
        double distance = DISTANCE_TO_LEFT_COL;
        double timeout = TIMEOUT_LEFT;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL;
            timeout = TIMEOUT_CENTER;
        }
        else if (cryptoColumn == RelicRecoveryVuMark.RIGHT) {
            distance = DISTANCE_TO_RIGHT_COL;
            timeout = TIMEOUT_RIGHT;
        }

        // Move backward out of balancing stone
        // Distance and timeout depends on column number;
        move(DT_POWER_FOR_STONE,
             DT_POWER_FOR_STONE,
             distance,
             false,
             timeout);

        // Move towards the crypto
        moveToCrypto(0, WAIT_FOR_JEWEL_FLICKER_MS);
    }
}
