package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    private static double TIMEOUT_RIGHT = 5;
    private static double TIMEOUT_CENTER = 6;
    private static double TIMEOUT_LEFT = 8;
    private static double DT_MOVE_TO_CRYPTO = 29.5;//inches
    private static final double DISTANCE_BACK_TO_CRYPTO_RF = 7;
    private static int DISTANCE_TO_RIGHT_COL_CM = 61;//cm
    private static int DISTANCE_TO_CENTER_COL_CM = 80;//43;//cm
    private static int DISTANCE_TO_LEFT_COL_CM = 97;//cm

    // Constructor
    public BoKAutoRedFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {
        far = true;

        // Detect Vuforia image, flick the jewel
        detectVuforiaImgAndFlick();

        // Move out of the balancing stone, distance
        moveRamp(DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, true, DT_TIMEOUT + 2);

        // turn left 90 degrees
        double current_angle = gyroTurn(DT_TURN_SPEED_HIGH, 0, TURN_LEFT_DEGREES, DT_TURN_TIMEOUT);

        // Move forwards towards cryptobox
        // Distance and timeout depends on column number
        int distance = DISTANCE_TO_RIGHT_COL_CM;
        double timeout = TIMEOUT_RIGHT;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL_CM;
            timeout = TIMEOUT_CENTER;
        }
        else if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            distance = DISTANCE_TO_LEFT_COL_CM;
            timeout = TIMEOUT_LEFT;
        }

        // Move towards the crypto
        moveWithRangeSensor(DT_POWER_FOR_RS, distance, false, timeout); // CM

        // Prepare to unload the glyph
        moveToCrypto();

        // Turn left 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, current_angle, 180, DT_TURN_TIMEOUT*3);

        // Deliver the glyph to crypto
        deliverGlyphToCrypto(DISTANCE_BACK_TO_CRYPTO_RF,
                             DISTANCE_AWAY_FROM_CRYPTO,
                             true, // jiggle the robot
                             0,    // reset the upper arm
                             -1);  // do not lower the wrist yet as it will hit the stone!
    }
}
