package org.firstinspires.ftc.teamcode;

import android.util.Log;

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

    private static final double DISTANCE_TO_LEFT_COL = 5.5;  // 21 // inches!!
    private static final double DISTANCE_TO_CENTER_COL = 3;  // 29
    private static final double DISTANCE_TO_RIGHT_COL = 9.5; // 37
    private static final double DISTANCE_BACK_TO_CRYPTO_BN = 9.25;

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
        detectVuforiaImgAndFlick();

        // Move backward out of balancing stone
        moveRamp(DT_POWER_FOR_STONE, DISTANCE_OFF_BALANCE, false, DT_TIMEOUT);

        // Move to the blue line
        moveUntilColor(DT_POWER_FOR_LINE, false, DT_TIMEOUT);

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

        // Distance and timeout depends on column number;
        if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            moveRamp(DT_POWER_FOR_STONE, distance, true, timeout);
        }
        else {
            moveRamp(DT_POWER_FOR_STONE, distance, false, timeout);
        }

        // Move towards the crypto
        moveToCrypto();

        // Turn left 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, 0, TURN_LEFT_DEGREES, DT_TURN_TIMEOUT);

        // Deliver glyph to crypto
        deliverGlyphToCrypto(DISTANCE_BACK_TO_CRYPTO_BN,
                DISTANCE_AWAY_FROM_CRYPTO,
                0,
                robot.wristInitPosFromFile);
    }
}
