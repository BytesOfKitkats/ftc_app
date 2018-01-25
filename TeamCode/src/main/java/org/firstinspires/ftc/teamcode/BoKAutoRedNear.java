package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedNear extends BoKAutoCommon {
    private static final double TIMEOUT_RIGHT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_LEFT = 6;

    private static final double DISTANCE_TO_RIGHT_COL = 5.75;  // 32.5 // inches!!
    private static final double DISTANCE_TO_CENTER_COL = 14;   // 40
    private static final double DISTANCE_TO_LEFT_COL = 21.25;  // 47
    private static final double DISTANCE_BACK_TO_CRYPTO_RN = 10.75;

    // Constructor
    public BoKAutoRedNear()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {

        // Detect Vuforia image and flick the jewel
        detectVuforiaImgAndFlick();

        // Move forward out of balancing stone
        moveRamp(DT_POWER_FOR_STONE, DISTANCE_OFF_BALANCE, true, DT_TIMEOUT);

        // Move to the red line
        moveUntilColor(DT_POWER_FOR_LINE, true, DT_TIMEOUT);

        // Distance and timeout depends on column number
        double distance = DISTANCE_TO_RIGHT_COL;
        double timeout = TIMEOUT_RIGHT;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL;
            timeout = TIMEOUT_CENTER;
        } else if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            distance = DISTANCE_TO_LEFT_COL;
            timeout = TIMEOUT_LEFT;
        }

        moveRamp(DT_POWER_FOR_STONE, distance, true, timeout);

        // Prepare to unload the glyph
        moveToCrypto();

        // Turn left 90 degrees
        gyroTurn(DT_TURN_SPEED_HIGH, 0, TURN_LEFT_DEGREES, DT_TURN_TIMEOUT);

        // Deliver glyph to crypto
        deliverGlyphToCrypto(DISTANCE_BACK_TO_CRYPTO_RN,
                DISTANCE_AWAY_FROM_CRYPTO,
                0,
                robot.wristInitPosFromFile);


    }
}
