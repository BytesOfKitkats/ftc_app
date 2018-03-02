package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    private static double TIMEOUT_RIGHT = 5;
    private static double TIMEOUT_CENTER = 6;
    private static double TIMEOUT_LEFT = 8;
    private static double DT_MOVE_TO_CRYPTO = 25.5;//inches
    private static int DISTANCE_TO_CENTER_COL_CM = 43;//cm
    private static int DISTANCE_TO_RIGHT_COL_CM = 24;//cm
    private static int DISTANCE_TO_LEFT_COL_CM = 63;//cm

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
        detectVuforiaImgAndFlick(WAIT_FOR_JEWEL_FLICKER_MS);

        // Move out of the balancing stone, distance
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, true, DT_TIMEOUT_6S);

        // turn left 90 degrees
        double currentAngle = gyroTurn(DT_TURN_SPEED_HIGH,
                                       0,
                                       TURN_LEFT_DEGREES,
                                       DT_TURN_THRESHOLD_LOW, // Threshold
                                       false, // NOT a tank turn
                                       false,
                                       DT_TURN_TIMEOUT);

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
        boolean success = moveWithRangeSensor(DT_POWER_FOR_RS, distance, false, timeout); // CM

        if(success) {
            // Prepare to unload the glyph
            moveToCrypto(currentAngle, WAIT_FOR_JEWEL_FLICKER_MS, false);
        }
        else
            robot.glyphArm.moveUpperArmDegrees(0, 0.4);
    }
}
