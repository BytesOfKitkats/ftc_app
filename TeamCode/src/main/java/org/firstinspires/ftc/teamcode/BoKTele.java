package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKTele
{
    // CONSTANTS
    private static final double GAME_STICK_DEAD_ZONE = 0.05;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double SPEED_COEFF_SLOW = 0.35;
    private static final double SPEED_COEFF_FAST = 0.9;
    private static final double SPEED_COEFF_TURN = 0.6;
    private static final int RA_JOYSTICK_RATIO = 350;
    private static final double RELIC_DEPLOY_POWER = 0.8;
    private static final double RELIC_RETRACT_POWER = 0.4;

    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = SPEED_COEFF_FAST;

    public enum BoKTeleStatus
    {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }

    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      BoKHardwareBot robot)
    {
        this.opMode = opMode;
        this.robot = robot;
        //robot.initializeImu();
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware()
    {
        boolean end_game = false;
        boolean relic_mode = false;

        boolean g2_left_bumper_pressed = false;
        boolean g2_right_bumper_pressed = false;
        boolean flipper_down = true;
        boolean bring_flipper_down = false;
        boolean g2_left_trigger_pressed = false;
        boolean g2_right_trigger_pressed = false;
        boolean flipper_lift_check = true;
        int flipperCount = -1;
        double roller_power = robot.ROLLER_POWER_HIGH;
        double FLIPPER_LIFT_POWER = 0.6;
        double FLIPPER_LIFT_POWER_LOW = 0.4;
        int FLIPPER_LIFT_LOW = 0;
        boolean ridingGatesLocked = true;

        robot.jewelArm.setPosition(robot.JA_INIT);
        robot.jewelFlicker.setPosition(robot.JF_INIT);
        robot.flipper.setPosition(robot.FLIPPER_DOWN_POS); // Flipper down
        robot.ridingGateLeft.setPosition(robot.RGL_LOCK);  // Gates up
        robot.ridingGateRight.setPosition(robot.RGR_LOCK);
        robot.leftRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.flipperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.initRelicArm();
        robot.relicArm.setPosition(robot.RA_INIT);
        robot.relicSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.relicSpool.setPower(0);

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // A:                  Go in slow mode
            // Y:                  Go in fast mode
            // Dpad Up:            Increase roller speed
            // Dpad Down:          Decrease roller speed

            moveRobot();

            if (opMode.gamepad1.y) {
                speedCoef = SPEED_COEFF_SLOW;
            }
            if (opMode.gamepad1.a) {
                speedCoef = SPEED_COEFF_FAST;
            }

            // GAMEPAD 2 CONTROLS
            // X:                      End Game
            // When in End Game, use
            // Dpad Down:              Start Relic Mode (hold Y for NOT deploying relic lift)
            // DPad Up:                End Relic Mode
            // Not relic mode:
            // A:                      Bring glyphs in
            // B:                      Stop intake motors
            // Y:					   Throw glyphs out
            // Left bumper:            Raise glyph flipper
            // Right bumper:           Lower glyph flipper
            // Left trigger:           Short burst of left roller motor reversed
            // Right trigger:          Short burst of right roller motor reversed
            // Relic Mode:
            // Left Stick:             Relic Lift
            // Right Stick:            Relic Arm
            // B:                      Relic claw open
            // A:                      Relic claw closed
            // Left bumper:            Raise relic arm to clear the wall
            // Right bumper:           Lower relic arm for relic placement

            if (opMode.gamepad2.x && !end_game) {
                end_game = true;
                Log.v("BOK", "End Game Started");
            }

            if (end_game) { // You need to be in end_game to be in relic mode
                if (opMode.gamepad2.dpad_down && !relic_mode) {
                    relic_mode = true;
                    // set mode to RUN_USING_ENCODER
                    //robot.setModeForDTMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                    speedCoef = SPEED_COEFF_SLOW;
                    // Make sure that the flipper lift is down
                    robot.flipperLift.setTargetPosition(FLIPPER_LIFT_LOW);
                    robot.flipperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.flipperLift.setPower(FLIPPER_LIFT_POWER);

					// Stop the intake rollers
                    robot.leftRoller.setPower(0);
                    robot.rightRoller.setPower(0);

                    // Make sure the roller gates are down
                    robot.ridingGateLeft.setPosition(robot.RGL_UNLOCK);
                    robot.ridingGateRight.setPosition(robot.RGR_UNLOCK);
                    ridingGatesLocked = false;

                    // turn the relic arm
                    robot.relicArm.setPosition(robot.RA_NEAR_POS);
                    g2_left_bumper_pressed = false;
                    g2_right_bumper_pressed = false;

                    Log.v("BOK", "Relic mode started");
                }

                if (opMode.gamepad2.dpad_up && relic_mode) {
                    relic_mode = false;
                    end_game = false;
                    robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    speedCoef = SPEED_COEFF_FAST;
                    robot.flipperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    //robot.relicArm.setPosition(robot.RA_INIT);
                    Log.v("BOK", "Relic mode ended");
                }
            }

            if (relic_mode) {
                // Relic lift deploying/retracting
                if ((opMode.gamepad2.left_stick_y) <= -GAME_TRIGGER_DEAD_ZONE) {
                    robot.relicSpool.setPower(-opMode.gamepad2.left_stick_y*RELIC_DEPLOY_POWER);
                    //Log.v("BOK","Relic Spool Position: " + robot.relicSpool.getCurrentPosition());
                } else if ((opMode.gamepad2.left_stick_y >= GAME_TRIGGER_DEAD_ZONE) &&
                        (robot.relicSpool.getCurrentPosition() >= 150)){
                    robot.relicSpool.setPower(-opMode.gamepad2.left_stick_y*RELIC_RETRACT_POWER);
                    //Log.v("BOK","Relic Spool Position: " + robot.relicSpool.getCurrentPosition());
                }
                else {
                    robot.relicSpool.setPower(0);
                }

                // Relic arm preset positions
                if (!g2_left_bumper_pressed && opMode.gamepad2.left_bumper ) {
                    //Log.v("BOK", "Left bumper pressed"); // raise the relic arm to RA_HIGH_POS
                    g2_left_bumper_pressed = true;
                    robot.relicArm.setPosition(robot.RA_RAISED_POS);
                }
                if (!g2_right_bumper_pressed && opMode.gamepad2.right_bumper ) {
                    //Log.v("BOK", "Right bumper pressed"); // lower the relic arm to RA_DEPLOY_POS
                    g2_right_bumper_pressed = true;
                    robot.relicArm.setPosition(robot.RA_FAR_POS);
                }
                // Relic arm rotation
                if (opMode.gamepad2.right_stick_y < -GAME_TRIGGER_DEAD_ZONE) {
                    double posOfArm = robot.relicArm.getPosition();
                    g2_left_bumper_pressed = false;
                    g2_right_bumper_pressed = false;

                    robot.relicArm.setPosition(posOfArm +
                            (-opMode.gamepad2.right_stick_y / RA_JOYSTICK_RATIO));

                    //Log.v("BOK", "Relic Arm pos: " + robot.relicArm.getPosition());
                } else if (opMode.gamepad2.right_stick_y > GAME_TRIGGER_DEAD_ZONE) {
                    // Move relic arm down
                    double posOfArm = robot.relicArm.getPosition();
                    g2_left_bumper_pressed = false;
                    g2_right_bumper_pressed = false;

                    robot.relicArm.setPosition(posOfArm -
                            (opMode.gamepad2.right_stick_y / RA_JOYSTICK_RATIO));

                    //Log.v("BOK", "Relic Arm pos: " + robot.relicArm.getPosition());
                }
                // Relic claw open/close
                if (opMode.gamepad2.b) { // open the relic claw
                    robot.relicClaw.setPosition(robot.RC_UNLOCK);
                }
                if (opMode.gamepad2.a) { // close the relic claw
                    robot.relicClaw.setPosition(robot.RC_LOCK);
                }
            } // if (relic_mode)
            else {
                // Roller motors
                if (opMode.gamepad2.y || opMode.gamepad2.dpad_up) {
                    // Spit out the glyphs by reversing roller motors
                    if (!flipper_down)
                        roller_power = robot.ROLLER_POWER_HIGH;
                    robot.leftRoller.setPower(-roller_power);
                    robot.rightRoller.setPower(-roller_power);
                }
                if (opMode.gamepad2.b) {
                    // Stop the roller motors
                    robot.leftRoller.setPower(0);
                    robot.rightRoller.setPower(0);
                }
                if (opMode.gamepad2.a || opMode.gamepad2.dpad_down) {
                    // Intake the glyphs
                    robot.leftRoller.setPower(roller_power);
                    robot.rightRoller.setPower(roller_power);
                }
                // Reverse the left roller as long as the left trigger is pressed
                if (opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
                    if (!g2_left_trigger_pressed) {
                        robot.rightRoller.setPower(0);
                        robot.leftRoller.setPower(-opMode.gamepad2.left_trigger);
                        g2_left_trigger_pressed = true;
                    }
                }
                else if (g2_left_trigger_pressed) { // reset
                    robot.rightRoller.setPower(roller_power);
                    robot.leftRoller.setPower(roller_power);
                    g2_left_trigger_pressed = false;
                }

                // Reverse the right roller as long as the right trigger is pressed
                if (opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE) {
                    if (!g2_right_trigger_pressed) {
                        robot.leftRoller.setPower(0);
                        robot.rightRoller.setPower(-opMode.gamepad2.right_trigger);
                        g2_right_trigger_pressed = true;
                    }
                }
                else if (g2_right_trigger_pressed) {// reset
                    robot.rightRoller.setPower(roller_power);
                    robot.leftRoller.setPower(roller_power);
                    g2_right_trigger_pressed = false;
                }

                // Flipper lift
                if (opMode.gamepad2.left_stick_y < -GAME_TRIGGER_DEAD_ZONE) {
                    // Raise the flipper lift
                    robot.flipperLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.flipperLift.setPower(FLIPPER_LIFT_POWER);
                    bring_flipper_down = false; // cancel the RUN_TO_POSITION
                    // Log.v("BOK", "Lift Pos " + robot.flipperLift.getCurrentPosition());
                } else if (opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE) {
                    if (!flipper_lift_check ||
                            (flipper_lift_check && robot.flipperLift.getCurrentPosition() > 0)) {
                        // Lower the lift
                        robot.flipperLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.flipperLift.setPower(-FLIPPER_LIFT_POWER);
                        bring_flipper_down = false; // cancel the RUN_TO_POSITION
                    }

                } else if (!bring_flipper_down && flipper_lift_check) {
                    // Hold the lift's last position, but the minimum is 100 so that the
                    // string remains tight.
                    //Log.v("BOK", "Flipper Lift check: " + robot.flipperLift.getCurrentPosition());
                    robot.flipperLift.setPower(0);
                    int currentLiftPosition = robot.flipperLift.getCurrentPosition();
                    if (currentLiftPosition < 0)
                        currentLiftPosition = FLIPPER_LIFT_LOW;
                    robot.flipperLift.setTargetPosition(currentLiftPosition);
                    robot.flipperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.flipperLift.setPower(FLIPPER_LIFT_POWER);
                }
                else if (!flipper_lift_check) {
                    robot.flipperLift.setPower(0);
                }

                // Flipper
                if (opMode.gamepad2.right_stick_y < -GAME_TRIGGER_DEAD_ZONE
                        && robot.flipper.getPosition() > robot.FLIPPER_UP_POS) {
                    // Raise the flipper
                    double pos = robot.flipper.getPosition();
                    //Log.v("BOK", "Flipper Up " + pos);
                    robot.flipper.setPosition(pos - 0.005); // flipper goes from 0.95 to 0.5
                    if (pos < robot.FLIPPER_ANGLE_POS) { // if the flipper is above a certain height
                        // Stop intake/egress of glyphs
                        roller_power = 0;
                        robot.leftRoller.setPower(roller_power);
                        robot.rightRoller.setPower(roller_power);
                    }
                } else if (opMode.gamepad2.right_stick_y > GAME_TRIGGER_DEAD_ZONE
                        && robot.flipper.getPosition() < robot.FLIPPER_DOWN_POS) {
                    // Lower the flipper
                    double pos = robot.flipper.getPosition();
                    //Log.v("BOK", "Flipper Down " + pos);
                    robot.flipper.setPosition(pos + 0.01); // flipper goes from 0.95 to 0.5
                    if (pos > robot.FLIPPER_ANGLE_POS) {
                        roller_power = robot.ROLLER_POWER_HIGH;
                    }
                }

                // LEFT BUMPER TO BRING THE FLIPPER DOWN CAREFULLY
                if (opMode.gamepad2.left_bumper && !bring_flipper_down) {
                    // Bring the flipper down, but first lower the flipper lift so that we don't
                    // hit the relic claw
                    bring_flipper_down = true;
                    // But first wait for the flipper lift to be lowered
                    robot.flipperLift.setTargetPosition(FLIPPER_LIFT_LOW);
                    robot.flipperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.flipperLift.setPower(FLIPPER_LIFT_POWER_LOW);
                }

                if (bring_flipper_down && !robot.flipperLift.isBusy()) {
                    // Once the flipper lift is lowered, then slowly bring the flipper down
                    // so that it doesn't hit the Misumi slide!
                    //robot.flipper.setPosition(robot.FLIPPER_DOWN_POS);
                    flipperCount = 10;
                    bring_flipper_down = false;
                }
                if (flipperCount > 0){
                    flipperCount--;
                    double pos = robot.flipper.getPosition();
                    if(pos < robot.FLIPPER_ANGLE_POS) {
                        robot.flipper.setPosition(pos + 0.005);
                    }
                    else{
                        flipperCount = 0;
                    }
                }
                else if (flipperCount == 0){
                    robot.flipper.setPosition(robot.FLIPPER_DOWN_POS);
                    flipperCount = -1;
                    if (!ridingGatesLocked) {
                        robot.ridingGateLeft.setPosition(robot.RGL_LOCK);
                        robot.ridingGateRight.setPosition(robot.RGR_LOCK);
                        ridingGatesLocked = true;
                    }
                    roller_power = robot.ROLLER_POWER_HIGH;
                    g2_right_bumper_pressed = false; // ready to dump again!
                }

                // RIGHT BUMPER TO DUMP
                if (opMode.gamepad2.right_bumper && !g2_right_bumper_pressed) {
                    // Bring the flipper up in order to dump the glyphs
                    // Unlock the gates
                    if (ridingGatesLocked) {
                        robot.ridingGateLeft.setPosition(robot.RGL_UNLOCK);
                        robot.ridingGateRight.setPosition(robot.RGR_UNLOCK);
                        opMode.sleep(robot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                        ridingGatesLocked = false;
                    }
                    // stop the rollers
                    roller_power = 0;
                    robot.leftRoller.setPower(roller_power);
                    robot.rightRoller.setPower(roller_power);

                    robot.flipper.setPosition(robot.FLIPPER_UP_POS);
                    g2_right_bumper_pressed = true;
                    Log.v ("BOK", "flipper up pos");
                }

                // If the flipper lift is higher than 175, stop the intake automatically
                if (!g2_right_bumper_pressed) {
                    int flipperLiftPos = Math.abs(robot.flipperLift.getCurrentPosition());
                    if (!flipper_down && flipperLiftPos <= 175) {
                        // Flipper lift is low, lock the gates
                        if (!ridingGatesLocked) {
                            robot.ridingGateLeft.setPosition(robot.RGL_LOCK);
                            robot.ridingGateRight.setPosition(robot.RGR_LOCK);
                            opMode.sleep(robot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                            ridingGatesLocked = true;
                        }
                        // set the roller power back to high
                        roller_power = robot.ROLLER_POWER_HIGH;
                        // Lower the flipper
                        robot.flipper.setPosition(robot.FLIPPER_DOWN_POS);
                        flipper_down = true;
                    }
                    else if (flipper_down && (flipperLiftPos >= 175)) {
                        // Flipper lift is high, stop intake & angle the flipper up
                        roller_power = 0;
                        robot.leftRoller.setPower(roller_power);
                        robot.rightRoller.setPower(roller_power);
                        robot.flipper.setPosition(robot.FLIPPER_ANGLE_POS);
                        flipper_down = false;
                    }
                }

                // Flipper gates
                if (opMode.gamepad2.dpad_left) {
                    if (!ridingGatesLocked) {
                        robot.ridingGateLeft.setPosition(robot.RGL_LOCK);
                        robot.ridingGateRight.setPosition(robot.RGR_LOCK);
                        ridingGatesLocked = true;
                    }
                }

                if (opMode.gamepad2.dpad_right) {
                    if (ridingGatesLocked) {
                        robot.ridingGateLeft.setPosition(robot.RGL_UNLOCK);
                        robot.ridingGateRight.setPosition(robot.RGR_UNLOCK);
                        ridingGatesLocked = false;
                    }
                }

                if (opMode.gamepad1.dpad_up) {
                    roller_power = robot.ROLLER_POWER_HIGH;
                }

                if (opMode.gamepad1.dpad_down) {
                    roller_power = robot.ROLLER_POWER_MID;
                }

                if (opMode.gamepad1.left_bumper) {
                    flipper_lift_check = false;
                    FLIPPER_LIFT_POWER = 0.3;
                    FLIPPER_LIFT_POWER_LOW = 0.3;
                }

                if (opMode.gamepad1.right_bumper) {
                    robot.flipperLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.flipperLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    flipper_lift_check = true;
                    FLIPPER_LIFT_POWER = 0.6;
                    FLIPPER_LIFT_POWER_LOW = 0.4;
                }
            }

            robot.waitForTick(robot.WAIT_PERIOD);
        }

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRobot()
    {
        /*
         * Gamepad1: Driver 1 controls the robot using the left joystick for throttle and
         * the right joystick for steering
         */
        // NOTE: the left joystick goes negative when pushed upwards
        double gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
        double gamePad1LeftStickX = opMode.gamepad1.left_stick_x;
        double gamePad1RightStickX = opMode.gamepad1.right_stick_x;

        if (speedCoef == SPEED_COEFF_FAST) {
            gamePad1LeftStickX = Math.pow(gamePad1LeftStickX, 3);
            gamePad1LeftStickY = Math.pow(gamePad1LeftStickY, 3);
        }

        double speedCoefLocal = speedCoef;
        double motorPowerLF = 0;
        double motorPowerLB = 0;
        double motorPowerRF = 0;
        double motorPowerRB = 0;

        //Log.v("BOK","moveRobot: " + String.format("%.2f", gamePad1LeftStickY) + ", " +
        //        String.format("%.2f", gamePad1LeftStickX) + ", " +
        //        String.format("%.2f", gamePad1RightStickX));

        // Run mecanum wheels

        if ((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) < -GAME_STICK_DEAD_ZONE)) {
            motorPowerLF = -gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerLB = -gamePad1LeftStickY - gamePad1LeftStickX;
            motorPowerRF = gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerRB = gamePad1LeftStickY - gamePad1LeftStickX;
            //Log.v("BOK","LF:" + String.format("%.2f", motorPowerLF*speedCoef) +
            //       "LB: " + String.format("%.2f", motorPowerLB*speedCoef) +
            //        "RF: " + String.format("%.2f", motorPowerRF*speedCoef) +
            //        "RB: " + String.format("%.2f", motorPowerRB*speedCoef));
        }
        else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) ||
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE)) {
            // Right joystick is for turning

            //first and last
            motorPowerLF = gamePad1RightStickX;
            motorPowerLB = gamePad1RightStickX;
            motorPowerRF = gamePad1RightStickX;
            motorPowerRB = gamePad1RightStickX;

            speedCoefLocal = SPEED_COEFF_TURN;
            //Log.v("BOK","Turn: LF:" + String.format("%.2f", motorPowerLF) +
            //        "LB: " + String.format("%.2f", motorPowerLB) +
            //        "RF: " + String.format("%.2f", motorPowerRF) +
            //        "RB: " + String.format("%.2f", motorPowerRB));
        }
        robot.setPowerToDTMotors(
                (motorPowerLF * speedCoefLocal),
                (motorPowerLB * speedCoefLocal),
                (motorPowerRF * speedCoefLocal),
                (motorPowerRB * speedCoefLocal));
    }
}
