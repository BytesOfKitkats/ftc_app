package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedNear extends BoKAutoCommon {
    private static final double TIMEOUT_RIGHT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_LEFT = 6;
    
    private static final double DISTANCE_TO_RIGHT_COL = 18; // inches!!
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
        // Detect Vuforia image and flick the jewel
        detectVuforiaImgAndDrop(WAIT_FOR_JEWEL_FLICKER_MS);
        moveAndFlick();

        // Move forward out of balancing stone
        // Distance and timeout depends on column number; TBD
        double distance = DISTANCE_TO_RIGHT_COL;
        double timeout = TIMEOUT_RIGHT;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL;
            timeout = TIMEOUT_CENTER;
        } else if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            distance = DISTANCE_TO_LEFT_COL;
            timeout = TIMEOUT_LEFT;
        }

        move(DT_POWER_FOR_STONE,
             DT_POWER_FOR_STONE,
             distance-distToMoveFlick,
             true,
             timeout);


        // Prepare to unload the glyph
        boolean secGlyph = true;
        moveToCrypto(0, WAIT_FOR_JEWEL_FLICKER_MS, secGlyph);
        if (secGlyph) {
            moveWColor(0.4, 100, true, 6);
            getSecondGlyph();
        }

    }
}
