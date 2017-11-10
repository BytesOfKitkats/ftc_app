package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueNear extends BoKAutoCommon
{
    private static final double TIMEOUT_RIGHT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_LEFT = 6;

    private static final double DISTANCE_TO_LEFT_COL = 26; // inches!!
    private static final double DISTANCE_TO_CENTER_COL = 33;
    private static final double DISTANCE_TO_RIGHT_COL = 40;

    @Override
    public void runSoftware()
    {
        // NOTE: Move backwards towards crypto
        // Detect Vuforia image
        if (getCryptoColumn(VUFORIA_TIMEOUT)) {
            // Setup flicker
            setJewelFlicker();
            opMode.sleep(WAIT_FOR_SERVO_MS);

            if (foundRedOnLeft) {
                robot.jewelFlicker.setPosition(robot.JF_LEFT);
                Log.v("BOK", "FOUND RED LEFT");
            } else {
                robot.jewelFlicker.setPosition(robot.JF_RIGHT);
                Log.v("BOK", "FOUND RED NOT ON LEFT");
            }
            opMode.sleep(WAIT_FOR_SERVO_MS);

            // Raise the flicker arm
            robot.jewelFlicker.setPosition(robot.JF_FINAL);
        }

        // Position the flicker to face the cryptobox
        robot.jewelArm.setPosition(robot.JA_INIT);

        // Move forward out of balancing stone
        // Distance and timeout depends on column number; TBD
        double distance = DISTANCE_TO_RIGHT_COL;
        double timeout = TIMEOUT_RIGHT;
        if (cryptoColumn == RelicRecoveryVuMark.CENTER) {
            distance = DISTANCE_TO_CENTER_COL;
            timeout = TIMEOUT_CENTER;
        }
        else if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            distance = DISTANCE_TO_LEFT_COL;
            timeout = TIMEOUT_LEFT;
        }

        // Move backward out of balancing stone
        // Distance and timeout depends on column number;
        move(DT_POWER_FOR_STONE,
             DT_POWER_FOR_STONE,
             distance,
             false,
             timeout);

        // Strafe to the right
        //strafe(DT_POWER_FOR_STRAFE,
        //       ROTATIONS_STRAFE_TO_WALL,
        //      true,
        //       DT_STRAFE_TIMEOUT);

        moveToBlueCrypto();
    }
}
