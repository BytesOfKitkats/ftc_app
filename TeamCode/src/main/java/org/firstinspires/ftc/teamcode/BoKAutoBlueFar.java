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
    private static double DT_MOVE_TO_CRYPTO = 22; // inches
    private static int DISTANCE_TO_LEFT_COL_CM = 33;
    private static int DISTANCE_TO_CENTER_COL_CM = 51;
    private static int DISTANCE_TO_RIGHT_COL_CM = 69;
    private static double DISTANCE_BACK_TO_CRYPTO = 7;

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
        detectVuforiaImgAndFlick();

        // Move back out of the balancing stone
        moveRamp(DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, false, DT_TIMEOUT);

        // Turn right 90 degrees
        double current_angle = gyroTurn(DT_TURN_SPEED_HIGH, 0, TURN_RIGHT_DEGREES, DT_TURN_TIMEOUT);

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
        moveToCrypto();

        //turn left 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, current_angle, 0, DT_TURN_TIMEOUT*3);

        deliverGlyphToCrypto(DISTANCE_BACK_TO_CRYPTO,
                             DISTANCE_AWAY_FROM_CRYPTO,
                             0,
                             robot.wristInitPosFromFile);
    }
}
