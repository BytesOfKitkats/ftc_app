package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueFar extends BoKAutoCommon
{
    private static int TURN_RIGHT_DEGREES = -90;
    private static double DT_MOVE_TO_CRYPTO = 22; // Inches! Must come off the balancing stone
    private static int DISTANCE_TO_LEFT_COL_CM = 51; // CM
    private static int DISTANCE_TO_CENTER_COL_CM = 68;
    private static int DISTANCE_TO_RIGHT_COL_CM = 87;

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
        detectVuforiaImgAndDrop(WAIT_FOR_JEWEL_FLICKER_MS);
        moveAndFlick();

        // Move back out of the balancing stone
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO-distToMoveFlick, false, DT_TIMEOUT_4S);

        gyroTurn(DT_TURN_SPEED_HIGH, 0, 0, DT_TURN_THRESHOLD_LOW, false, false, DT_TIMEOUT_2S);

        double cmFromWall = robot.getDistanceCM(robot.mb1240Back);
        if(cryptoColumn == RelicRecoveryVuMark.LEFT)
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE,
                                  DISTANCE_TO_LEFT_COL_CM, true, DT_TIMEOUT_5S);
        else if (cryptoColumn == RelicRecoveryVuMark.CENTER)
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE,
                                  DISTANCE_TO_CENTER_COL_CM, true, DT_TIMEOUT_5S);
        else
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE,
                                  DISTANCE_TO_RIGHT_COL_CM, true, DT_TIMEOUT_6S);

        double distBack = (cmFromWall - 20.32)/2.54; // 20.32cm = 8 inches
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, distBack, false, DT_TIMEOUT_4S);

        flipFlipper(FLIP_FLIPPER_DUMP); // dump the glyph

        // just park in the safe zone
        moveRamp(DT_POWER_HIGH, distBack/2, true, DT_TIMEOUT_4S);
    }
}
