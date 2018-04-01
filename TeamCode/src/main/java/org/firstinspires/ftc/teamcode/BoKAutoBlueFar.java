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
        detectVuforiaImgAndFlick(WAIT_FOR_JEWEL_FLICKER_MS);

        // Move back out of the balancing stone
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, DT_MOVE_TO_CRYPTO, false, DT_TIMEOUT_4S);

        gyroTurn(DT_TURN_SPEED_HIGH, 0, 0, DT_TURN_THRESHOLD_LOW, false, false, DT_TIMEOUT_2S);

        double cmFromWall = robot.getDistanceCM(robot.mb1240Back);
        if(cryptoColumn == RelicRecoveryVuMark.LEFT)
            strafeWithRangeSensor(0.2, 51, true, 5);
        else if (cryptoColumn == RelicRecoveryVuMark.CENTER)
            strafeWithRangeSensor(0.2, 67, true, 5);
        else
            strafeWithRangeSensor(0.2, 86, true, 6);

        double distBack = (cmFromWall - 20.32)/2.54; // 20.32cm = 8 inches
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, distBack, false, DT_TIMEOUT_4S);

        flipFlipper(2);

        // just park in the safe zone
        moveRamp(DT_POWER_HIGH, distBack, true, DT_TIMEOUT_4S);
    }
}
