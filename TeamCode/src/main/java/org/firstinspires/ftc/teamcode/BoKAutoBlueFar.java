package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueFar extends BoKAutoCommon
{
    private static double TIMEOUT_LEFT = 6;
    private static double TIMEOUT_CENTER = 8;
    private static double TIMEOUT_RIGHT = 10;
    private static int TURN_RIGHT_DEGREES = -90;
    private static double DT_MOVE_TO_CRYPTO = 22; // Inches! Must come off the balancing stone
    private static int DISTANCE_TO_LEFT_COL_CM = 44; // CM
    private static int DISTANCE_TO_CENTER_COL_CM = 62;
    private static int DISTANCE_TO_RIGHT_COL_CM = 80;

    // Constructor
    public BoKAutoBlueFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        // NOTE: Move backwards towards crypto
        far = true;

        // Detect Vuforia image and flick the jewel
        detectVuforiaImgAndFlick(WAIT_FOR_JEWEL_FLICKER_MS_LOW);

        // Move back out of the balancing stone
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, false, DT_TIMEOUT_4S);

        // Turn right 90 degrees
        double currentAngle = gyroTurn(DT_TURN_SPEED_LOW,
                                       0,
                                       TURN_RIGHT_DEGREES,
                                       DT_TURN_THRESHOLD_LOW,
                                       false,
                                       false,
                                       DT_TURN_TIMEOUT);

        move(DT_POWER_HIGH, DT_POWER_HIGH, 5, false, DT_TIMEOUT_2S);

        // Make a quick tank turn in order to get some space for the flicker
        // Instead of strafing!!
        currentAngle = gyroTurn(DT_TURN_SPEED_HIGH,
                                currentAngle,
                                -45, // turn left
                                DT_TURN_THRESHOLD_HIGH,
                                true,  // Tank turn
                                false, // turn right
                                DT_TURN_TIMEOUT/2);
        // And reset back to 90 degrees!
        currentAngle = gyroTurn(DT_TURN_SPEED_HIGH,
                                currentAngle,
                                TURN_RIGHT_DEGREES,
                                DT_TURN_THRESHOLD_HIGH,
                                false, // NOT a tank turn
                                false,
                                DT_TURN_TIMEOUT);

        //strafeRamp(0.5, 0.5, false, 2);
        //gyroTurn(DT_TURN_SPEED_HIGH, currentAngle, TURN_RIGHT_DEGREES, false, false, DT_TURN_TIMEOUT);

        // Distance to crypto and timeout depends on column number
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
        moveWithRangeSensor(DT_POWER_FOR_RS, distance, true, timeout); // CM

        // Prepare to unload the glyph
        moveToCrypto(currentAngle, WAIT_FOR_JEWEL_FLICKER_MS_LOW, false);
    }
}
