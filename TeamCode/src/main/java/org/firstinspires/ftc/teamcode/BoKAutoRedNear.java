package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedNear extends BoKAutoCommon {
    private static final double TIMEOUT_RIGHT = 4;
    private static final double TIMEOUT_CENTER = 5;
    private static final double TIMEOUT_LEFT = 6;
    
    private static final double DISTANCE_TO_RIGHT_COL = 16.5; // inches!!
    private static final double DISTANCE_TO_CENTER_COL = 26;
    private static final double DISTANCE_TO_LEFT_COL = 33;

    @Override
    public void runSoftware() {
        // Detect Vuforia image
        getCryptoColumn(VUFORIA_TIMEOUT);
        // Setup flicker
        setJewelFlicker();
        opMode.sleep(WAIT_FOR_SERVO_MS);

        if (foundRedOnLeft) {
            robot.jewelFlicker.setPosition(robot.JF_RIGHT);
        } else {
            robot.jewelFlicker.setPosition(robot.JF_LEFT);
        }
        opMode.sleep(WAIT_FOR_SERVO_MS);

        // Raise the flicker arm and position the flicker to face the cryptobox
        robot.jewelFlicker.setPosition(robot.JF_FINAL);
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

        move(DT_POWER_FOR_STONE, 
             DT_POWER_FOR_STONE, 
             distance,
             true, 
             timeout);

        // Strafe to the right
        strafe(DT_POWER_FOR_STRAFE, 
               ROTATIONS_STRAFE_TO_WALL, 
               true, 
               DT_STRAFE_TIMEOUT);
               
        moveRedCrypto(); 
    }
}
