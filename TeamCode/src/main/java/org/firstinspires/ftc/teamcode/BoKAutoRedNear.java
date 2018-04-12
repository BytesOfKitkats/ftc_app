package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

        InitRelicArmThread initRA = new InitRelicArmThread();
        initRA.start();

        move(DT_POWER_FOR_STONE,
             DT_POWER_FOR_STONE,
             distance-distToMoveFlick,
             true,
             timeout);

        // Prepare to unload the glyph
        moveToCrypto(0, WAIT_FOR_JEWEL_FLICKER_MS, secGlyph);
        if (secGlyph) {
            int INIT_DISTANCE_FORWARD = 15;
            double DISTANCE_BACK_PART_1 = 10.5;
            double DISTANCE_BACK_PART_2 = 2.5;
            int FINAL_DISTANCE_FORWARD = 6;
            int MAX_DISTANCE_TO_COLOR = 24;
            double MOVE_TO_LINE_POWER_HIGH = 0.25;
            double MOVE_TO_LINE_POWER_LOW = 0.2;
            double MOVE_BACK_POWER = 0.3;
            double TURN_POWER_VLOW = 0.16;
            int MOVE_BACK_TIMEOUT = 3;

            // move at a high speed past the safe zone triangle
            move(DT_POWER_HIGH, DT_POWER_HIGH, INIT_DISTANCE_FORWARD, true, DT_TIMEOUT_4S);

            // setup the flipper and the flipper gates
            robot.flipper.setPosition(FLIP_FLIPPER_LOWER);
            moveFlipperGates(true);
            // move slowly till the blue line, records encCountsTillLine
            moveWColor(MOVE_TO_LINE_POWER_HIGH, MAX_DISTANCE_TO_COLOR, true, DT_TIMEOUT_4S);

            // get second (& third) glyphs
            getSecondGlyph();

            // move slowly back to the blue line
            moveWColor(MOVE_TO_LINE_POWER_LOW, MAX_DISTANCE_TO_COLOR, false, DT_TIMEOUT_4S);

            // we may have rotated a bit while in the glyph pit, so turn the robot back
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                                                     AxesOrder.XYZ,
                                                     AngleUnit.DEGREES);
            gyroTurn(TURN_POWER_VLOW,
                     angles.thirdAngle, // current angle of the robot
                     TURN_LEFT_DEGREES, // turn angle at the end of moveToCrypto
                     DT_TURN_THRESHOLD_LOW, false, false, DT_TIMEOUT_2S);

            // now go back to the crypto box by retracing the steps
            robot.resetDTEncoders();
            robot.startEncMove(MOVE_BACK_POWER, MOVE_BACK_POWER, encCountsTillLine, false);
            // Keep the rollers in the intake mode, so that we can jiggle the glyphs
            robot.leftRoller.setPower(robot.ROLLER_POWER_HIGH);
            robot.rightRoller.setPower(robot.ROLLER_POWER_HIGH);

            runTime.reset();
            while (opMode.opModeIsActive() && robot.areDTMotorsBusy()) {
                if (runTime.seconds() >= MOVE_BACK_TIMEOUT) {
                    Log.v("BOK", "Move back timed out!");
                    break;
                }
            }
            robot.stopMove();
            robot.leftRoller.setPower(0);
            robot.rightRoller.setPower(0);

            if(numGlyphs > 0) {
                // Split the INIT_DISTANCE_FORWARD, go back DISTANCE_BACK_PART_1
                moveRamp(DT_POWER_HIGH, DISTANCE_BACK_PART_1, false, DT_TIMEOUT_4S);
                Log.v("BOK", "Ready to flip, raise the lift");
                // Raise the flipper lift
                flipFlipper(RAISE_LIFT);
                BoKAutoBlueNear.FlipFlipperThread dumpGlyphs = new FlipFlipperThread();
                dumpGlyphs.start();
                // Move back a bit and push the glyphs in
                moveRamp(MOVE_BACK_POWER, DISTANCE_BACK_PART_2, false, DT_TIMEOUT_2S);
                try {
                    dumpGlyphs.join();
                } catch (InterruptedException e) {

                }
                opMode.sleep(250);

                // Go and park in the safe zone
                move(DT_POWER_HIGH, DT_POWER_HIGH, FINAL_DISTANCE_FORWARD, true, DT_TIMEOUT_2S);
                // Lower the flipper lift and the flipper
                flipFlipper(LOWER_LIFT_AND_RESET_FLIPPER);
            }
            else { // No glyphs! go back & park in the safe zone
                moveRamp(DT_POWER_HIGH, (INIT_DISTANCE_FORWARD-5), false, DT_TIMEOUT_4S);
            }
        } // if (SecGlyph)
    }
}
