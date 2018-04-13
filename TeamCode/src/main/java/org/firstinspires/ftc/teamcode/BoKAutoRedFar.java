package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    private static double TIMEOUT_DUMP = 1;
    private static double DT_MOVE_TO_CRYPTO = 25.5;//inches
    private static int DISTANCE_TO_RIGHT_COL_CM = 51;//cm
    private static int DISTANCE_TO_CENTER_COL_CM = 68;//cm
    private static int DISTANCE_TO_LEFT_COL_CM = 89;//cm

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
        detectVuforiaImgAndDrop(WAIT_FOR_JEWEL_FLICKER_MS);
        moveAndFlick();

        InitRelicArmThread initRA = new InitRelicArmThread();
        initRA.start();

        // Move out of the balancing stone, distance
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE,
                DT_MOVE_TO_CRYPTO-distToMoveFlick, true, DT_TIMEOUT_6S);

        double cmFromWall = robot.getDistanceCM(robot.mb1240Front);
        if(cryptoColumn == RelicRecoveryVuMark.LEFT)
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE,
                                  DISTANCE_TO_LEFT_COL_CM, DT_TIMEOUT_6S);
        else if (cryptoColumn == RelicRecoveryVuMark.CENTER)
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE,
                                  DISTANCE_TO_CENTER_COL_CM, DT_TIMEOUT_5S);
        else
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE,
                                  DISTANCE_TO_RIGHT_COL_CM, DT_TIMEOUT_5S);

        double distBack = (cmFromWall - 20.32)/2.54; // 20.32cm = 8 inches
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, distBack, true, DT_TIMEOUT_4S);

        flipFlipper(FLIP_FLIPPER_LOWER); // lower the flipper
        // dump the glyph by reversing the rollers
        moveRollers(REVERSE_ROLLERS, robot.ROLLER_POWER_HIGH, TIMEOUT_DUMP, runTime);

        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE, distBack/2, false, DT_TIMEOUT_4S);
    }
}
