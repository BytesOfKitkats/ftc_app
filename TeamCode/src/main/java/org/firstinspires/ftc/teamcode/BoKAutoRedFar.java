package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    private static double TIMEOUT_DUMP = 1;
    private static double DT_MOVE_TO_CRYPTO = 25.5;//inches
    private static int DISTANCE_TO_RIGHT_COL_CM = 49;//cm
    private static int DISTANCE_TO_CENTER_COL_CM = 66;//cm
    private static int DISTANCE_TO_LEFT_COL_CM = 86;//cm

    // Constructor
    public BoKAutoRedFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    class RaiseFlipperThread extends Thread
    {
        @Override
        public void run() {
            robot.flipper.setPosition(robot.FLIPPER_ANGLE_POS-0.05);
        }
    }

    @Override
    public void runSoftware()
    {
        far = true;

        // Detect Vuforia image, flick the jewel
        detectVuforiaImgAndDrop(WAIT_FOR_JEWEL_FLICKER_MS);
        RaiseFlipperThread th = new RaiseFlipperThread();
        th.start();
        moveAndFlick();

        InitRelicArmThread initRA = new InitRelicArmThread();
        initRA.start();

        // Move out of the balancing stone, distance
        move(DT_POWER_FOR_STONE, DT_POWER_FOR_STONE,
                DT_MOVE_TO_CRYPTO-distToMoveFlick, true, DT_TIMEOUT_6S);
        opMode.sleep(100);
        double cmFromWall = robot.getDistanceCM(robot.mb1240Front);
        Log.v("BOK", "CM from wall " + cmFromWall);

        if (cmFromWall > 35) {
            cmFromWall = robot.getDistanceCM(robot.mb1240Front);
            Log.v("BOK", "CM from wall 2 " + cmFromWall);
            if (cmFromWall > 35) cmFromWall = 35;
        }

        additional_turn_speed = 0.5;
        fraction_turn_slow = 0.2;
        gyroTurn(DT_TURN_SPEED_HIGH, 0, 180, DT_TURN_THRESHOLD_LOW, false, false, DT_TIMEOUT_6S);

        if(cryptoColumn == RelicRecoveryVuMark.LEFT)
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE*1.5,
                                  DISTANCE_TO_LEFT_COL_CM, false, DT_TIMEOUT_6S);
        else if (cryptoColumn == RelicRecoveryVuMark.CENTER)
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE*1.5,
                                  DISTANCE_TO_CENTER_COL_CM, false, DT_TIMEOUT_5S);
        else
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE*1.5,
                                  DISTANCE_TO_RIGHT_COL_CM, false, DT_TIMEOUT_5S);

        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES);
        fraction_turn_slow = 0.25;
        gyroTurn(DT_TURN_SPEED_LOW, angles.thirdAngle, 180, DT_TURN_THRESHOLD_LOW, false, false, DT_TIMEOUT_4S);

        if (cryptoColumn == RelicRecoveryVuMark.LEFT) {
            cmFromWall = robot.getDistanceCM(robot.mb1240Back);
            if (cmFromWall > 50) {
                cmFromWall = robot.getDistanceCM(robot.mb1240Back);
                if (cmFromWall > 50) cmFromWall = 50;
            }
        }
        Log.v("BOK", "CM from wall " + cmFromWall);

        double distBack = (cmFromWall - 10.16)/2.54; // 20.32cm = 8 inches
        flipFlipper(FLIP_FLIPPER_DUMP); // dump the flipper
        Log.v("BOK", "Inches from wall: " + String.format("%.1f", distBack));
        move(DT_POWER_HIGH, DT_POWER_HIGH, distBack, false, DT_TIMEOUT_4S);

        if (!secGlyph) {
            // just park in the safe zone
            moveRamp(DT_POWER_HIGH, distBack / 2 + 1, true, DT_TIMEOUT_4S);
        }
        else {
            int INIT_DISTANCE_FORWARD = 20;
            double DISTANCE_BACK_PART_1 = 20;
            double DISTANCE_BACK_PART_2 = distBack-5;
            int FINAL_DISTANCE_FORWARD = 6;
            int MAX_DISTANCE_TO_COLOR = 24;
            int GYRO_TURN_DEGREES = 150;
            double MOVE_TO_LINE_POWER_HIGH = 0.25;
            double MOVE_TO_LINE_POWER_LOW = 0.2;
            double TURN_POWER_VLOW = 0.15;
            double MOVE_BACK_POWER = 0.3;
            int MOVE_BACK_TIMEOUT = 3;

            // move forward and strafe
            move(DT_POWER_HIGH, DT_POWER_HIGH, distBack, true, DT_TIMEOUT_4S);
            if (cryptoColumn != RelicRecoveryVuMark.LEFT) {
                // First strafe to the right column
                strafeWithRangeSensor(DT_POWER_FOR_STRAFE_HIGH*2,
                        DISTANCE_TO_LEFT_COL_CM, false, DT_TIMEOUT_6S);
            }

            // Turn slightly to the right
            additional_turn_speed = 0.5;
            gyroTurn(DT_TURN_SPEED_HIGH, angles.thirdAngle, GYRO_TURN_DEGREES,
                    DT_TURN_THRESHOLD_LOW, false, false, DT_TIMEOUT_2S);
            additional_turn_speed = 0;

            // move at a high speed past the safe zone triangle
            move(DT_POWER_HIGH*2, DT_POWER_HIGH*2, INIT_DISTANCE_FORWARD, true, DT_TIMEOUT_4S);

            // setup the flipper and the flipper gates
            robot.flipper.setPosition(FLIP_FLIPPER_LOWER);
            moveFlipperGates(true);
            // move slowly till the red line, records encCountsTillLine
            moveWColor(MOVE_TO_LINE_POWER_HIGH, MAX_DISTANCE_TO_COLOR, true, DT_TIMEOUT_4S);

            // get second (& third) glyphs
            getSecondGlyph(0.3, 0.6);

            // move slowly back to the red line
            moveWColor(MOVE_TO_LINE_POWER_LOW, MAX_DISTANCE_TO_COLOR, false, DT_TIMEOUT_4S);

            // we may have rotated a bit while in the glyph pit, so turn the robot back
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES);
            gyroTurn(DT_TURN_SPEED_HIGH,
                    angles.thirdAngle, // current angle of the robot
                    GYRO_TURN_DEGREES+5,
                    DT_TURN_THRESHOLD_LOW, false, false, DT_TIMEOUT_4S);

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

            // Split the INIT_DISTANCE_FORWARD, go back DISTANCE_BACK_PART_1
            moveRamp(DT_POWER_HIGH+0.1, DISTANCE_BACK_PART_1, false, DT_TIMEOUT_4S);

            // Turn again to be parallel to the wall
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES);
            gyroTurn(DT_TURN_SPEED_HIGH,
                    angles.thirdAngle, // current angle of the robot
                    180,
                    DT_TURN_THRESHOLD_LOW, false, false, DT_TIMEOUT_4S);

            // Strafe
            strafeWithRangeSensor(DT_POWER_FOR_STRAFE,
                    DISTANCE_TO_LEFT_COL_CM, false, DT_TIMEOUT_6S);

            robot.leftRoller.setPower(0);
            robot.rightRoller.setPower(0);

            double distNear = robot.distNear.getDistance(DistanceUnit.CM);
            double distFar = robot.distFar.getDistance(DistanceUnit.CM);
            boolean glyphFound = !Double.isNaN(distNear) ||
                    !Double.isNaN(distFar);

            if (glyphFound)
                numGlyphs = 1;

            if(numGlyphs > 0) {
                Log.v("BOK", "Ready to flip, raise the lift");
                //moveRamp(MOVE_BACK_POWER, DISTANCE_BACK_PART_2, false, DT_TIMEOUT_2S);

                // Raise the flipper lift (if there is a glyph there)
                if (cryptoColumn == RelicRecoveryVuMark.LEFT)
                    flipFlipper(RAISE_LIFT);
                // Dump glyphs
                FlipFlipperThread dumpGlyphs = new FlipFlipperThread();
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

                //moveRamp(DT_POWER_HIGH, (INIT_DISTANCE_FORWARD-5), false, DT_TIMEOUT_4S);
            }
        }
    }
}
