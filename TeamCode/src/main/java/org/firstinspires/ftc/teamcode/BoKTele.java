package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKTele
{
    // CONSTANTS
    private static final double GAME_STICK_DEAD_ZONE = 0.05;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double TURNTABLE_STICK_DEAD_ZONE = 0.8;
    private static final double UPPER_ARM_STICK_DEAD_ZONE = 0.2;
    private static final double TURNTABLE_MOTOR_POWER = 0.2;
    private static final int TURNTABLE_COUNTS_PER_MOTOR_REV = 1120; // AndyMark 40
    private static final double UPPER_ARM_MOTOR_POWER_SLOW = 0.4;//0.2
    private static final double UPPER_ARM_MOTOR_POWER_FAST = 0.8;//0.4
    private static final double SPEED_COEFF_SLOW = 0.35;
    private static final double SPEED_COEFF_FAST = 0.9;
    private static final int RA_JOYSTICK_RATIO = 200;
    private static final double RELIC_DEPLOY_POWER = 0.6;
    private static final double RELIC_RETRACT_POWER = 0.4;
    private static final float GLYPH_FLICKER_INCREMENT = 0.01F;

    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = SPEED_COEFF_FAST;
    private boolean trigger_left_decrease = false;

    public enum BoKTeleStatus
    {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }

    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      BoKHardwareBot robot,
                                      boolean trigger_left_decrease)
    {
        this.trigger_left_decrease = trigger_left_decrease;
        this.opMode = opMode;
        this.robot = robot;
        //robot.initializeImu();
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware()
    {
        boolean tank = false;
        boolean end_game = false;
        boolean relic_mode = false;
        boolean placementMode = false;

        boolean g1_left_bumper_pressed = false;
        boolean g2_left_bumper_pressed = false;
        boolean g2_right_bumper_pressed = false;
        //boolean g2_left_trigger_pressed = false;
        //boolean glyph_at_end = false;
        boolean relic_deploying = false;
        boolean flipper_down = true;

        double FLIPPER_LIFT_POWER = 0.4;


        robot.jewelArm.setPosition(robot.JA_INIT);
        robot.jewelFlicker.setPosition(robot.JF_INIT);
        robot.leftRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.flipperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // A:                  Go in slow mode
            // Y:                  Go in fast mode

            moveRobot();

            if (opMode.gamepad1.y) {
                speedCoef = SPEED_COEFF_SLOW;
            }
            if (opMode.gamepad1.a) {
                speedCoef = SPEED_COEFF_FAST;
            }

            // GAMEPAD 2 CONTROLS
            // Y:                      End Game
            // When in End Game, use
            // Dpad Down:              Start Relic Mode (hold Y for NOT deploying relic lift)
            // DPad Up:                End Relic Mode
            // Not relic mode:
            // A:                      Bring glyphs in
            // B:                      Stop intake motors
            // X:					   Throw glyphs out
            // Left bumper:            Raise glyph flipper
            // Right bumper:           Lower glyph flipper
            // Relic Mode:
            // Left Stick:             Relic Lift
            // Right Stick:            Relic Arm
            // B:                      Relic claw open
            // A:                      Relic claw closed
            // Left bumper:            Raise relic arm to clear the wall
            // Right bumper:           Lower relic arm for relic placement

            if (opMode.gamepad2.y && !end_game) {
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
                    robot.flipperLift.setTargetPosition(100);
                    robot.flipperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.flipperLift.setPower(FLIPPER_LIFT_POWER);
					// Stop the intake rollers
                    robot.leftRoller.setPower(0);
                    robot.rightRoller.setPower(0);
                    // Make sure the roller gates are down
                    robot.ridingGateLeft.setPosition(robot.RGL_UNLOCK);
                    robot.ridingGateRight.setPosition(robot.RGR_UNLOCK);

                    // turn the relic arm
                    robot.relicArm.setPosition(robot.RA_NEAR_POS);
                    g2_left_bumper_pressed = false;
                    g2_right_bumper_pressed = false;

                    Log.v("BOK", "Relic mode started");
                }

                if (opMode.gamepad2.dpad_up) {
                    relic_mode = false;
                    end_game = false;
                    relic_deploying = false;
                    robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    speedCoef = SPEED_COEFF_FAST;
                    robot.flipperLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.flipperLift.setPower(0);
                    //robot.relicArm.setPosition(robot.RA_INIT);
                    Log.v("BOK", "Relic mode ended");
                }
            }

            if (relic_mode) {
                if ((opMode.gamepad2.left_stick_y) <= -GAME_TRIGGER_DEAD_ZONE) {
                    relic_deploying = false; // manual control enabled!
                    robot.relicSpool.setPower(-opMode.gamepad2.left_stick_y*RELIC_DEPLOY_POWER);
                    //Log.v("BOK","Relic Spool Position: " + robot.relicSpool.getCurrentPosition());
                } else if (opMode.gamepad2.left_stick_y >= GAME_TRIGGER_DEAD_ZONE) {
                    relic_deploying = false; // manual control enabled!
                    robot.relicSpool.setPower(-opMode.gamepad2.left_stick_y*RELIC_RETRACT_POWER);
                    //Log.v("BOK","Relic Spool Position: " + robot.relicSpool.getCurrentPosition());
                }
                else if (!relic_deploying) { // manual mode
                    robot.relicSpool.setPower(0);
                }
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
                if (opMode.gamepad2.right_stick_y < -UPPER_ARM_STICK_DEAD_ZONE) {
                    double posOfArm = robot.relicArm.getPosition();
                    g2_left_bumper_pressed = false;
                    g2_right_bumper_pressed = false;

                    robot.relicArm.setPosition(posOfArm +
                            (-opMode.gamepad2.right_stick_y / RA_JOYSTICK_RATIO));

                    Log.v("BOK", "Relic Arm pos: " + robot.relicArm.getPosition());
                } else if (opMode.gamepad2.right_stick_y > UPPER_ARM_STICK_DEAD_ZONE) {
                    // Move relic arm down
                    double posOfArm = robot.relicArm.getPosition();
                    g2_left_bumper_pressed = false;
                    g2_right_bumper_pressed = false;

                    robot.relicArm.setPosition(posOfArm -
                            (opMode.gamepad2.right_stick_y / RA_JOYSTICK_RATIO));

                    Log.v("BOK", "Relic Arm pos: " + robot.relicArm.getPosition());
                }
                if (opMode.gamepad2.b) { // open the relic claw
                    robot.relicClaw.setPosition(robot.RC_UNLOCK);
                }
                if (opMode.gamepad2.a) { // close the relic claw
                    robot.relicClaw.setPosition(robot.RC_LOCK);
                }
            } // if (relic_mode)
            else {
                if (opMode.gamepad2.x) {
                    robot.leftRoller.setPower(-robot.ROLLER_POWER);
                    robot.rightRoller.setPower(-robot.ROLLER_POWER);
                }
                if (opMode.gamepad2.b) {
                    robot.leftRoller.setPower(0);
                    robot.rightRoller.setPower(0);
                }
                if (opMode.gamepad2.a) {
                    robot.leftRoller.setPower(robot.ROLLER_POWER);
                    robot.rightRoller.setPower(robot.ROLLER_POWER);
                }
                if (opMode.gamepad2.left_stick_y < -UPPER_ARM_STICK_DEAD_ZONE) {
                    robot.ridingGateLeft.setPosition(robot.RGL_LOCK);
                    robot.ridingGateRight.setPosition(robot.RGR_LOCK);
                    robot.flipperLift.setPower(FLIPPER_LIFT_POWER);
                } else if (opMode.gamepad2.left_stick_y > UPPER_ARM_STICK_DEAD_ZONE &&
                        robot.flipperLift.getCurrentPosition() > 0) {
                    robot.flipperLift.setPower(-FLIPPER_LIFT_POWER);
                } else {
                    robot.flipperLift.setPower(0);
                }

                if (opMode.gamepad2.right_stick_y < -UPPER_ARM_STICK_DEAD_ZONE
                        && robot.flipper.getPosition() > 0.5) {
                    double pos = robot.flipper.getPosition();
                    //Log.v("BOK", "Flipper Up " + pos);
                    robot.flipper.setPosition(pos - 0.005);
                } else if (opMode.gamepad2.right_stick_y > UPPER_ARM_STICK_DEAD_ZONE
                        && robot.flipper.getPosition() < 0.95) {
                    double pos = robot.flipper.getPosition();
                    //Log.v("BOK", "Flipper Down " + pos);
                    robot.flipper.setPosition(pos + 0.005);
                }

                if (opMode.gamepad2.left_bumper) {
                    robot.flipper.setPosition(robot.FLIPPER_DOWN_POS);
                    g2_right_bumper_pressed = false;
                    //Log.v ("BOK", "flipper down pos");
                }

                if (opMode.gamepad2.right_bumper) {
                    robot.ridingGateLeft.setPosition(robot.RGL_UNLOCK);
                    robot.ridingGateRight.setPosition(robot.RGR_UNLOCK);
                    opMode.sleep(robot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                    robot.flipper.setPosition(robot.FLIPPER_UP_POS);
                    g2_right_bumper_pressed = true;
                    //Log.v ("BOK", "flipper up pos");
                }
                if (!g2_right_bumper_pressed && flipper_down &&
                        Math.abs((robot.flipperLift.getCurrentPosition())) >= 175) {
                    robot.flipper.setPosition(robot.FLIPPER_ANGLE_POS);
                    flipper_down = false;
                    g2_right_bumper_pressed = false;
                } else if (!g2_right_bumper_pressed && !flipper_down &&
                        Math.abs((robot.flipperLift.getCurrentPosition())) <= 175) {
                    robot.ridingGateLeft.setPosition(robot.RGL_LOCK);
                    robot.ridingGateRight.setPosition(robot.RGR_LOCK);
                    opMode.sleep(robot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
                    robot.flipper.setPosition(robot.FLIPPER_DOWN_POS);
                    g2_right_bumper_pressed = false;
                    flipper_down = true;
                }

                if (opMode.gamepad2.dpad_left) {
                    robot.ridingGateLeft.setPosition(robot.RGL_LOCK);
                    robot.ridingGateRight.setPosition(robot.RGR_LOCK);
                }

                if (opMode.gamepad2.dpad_right) {
                    robot.ridingGateLeft.setPosition(robot.RGL_UNLOCK);
                    robot.ridingGateRight.setPosition(robot.RGR_UNLOCK);
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

            //Log.v("BOK","Turn: LF:" + String.format("%.2f", motorPowerLF) +
            //        "LB: " + String.format("%.2f", motorPowerLB) +
            //        "RF: " + String.format("%.2f", motorPowerRF) +
            //        "RB: " + String.format("%.2f", motorPowerRB));
        }
        robot.setPowerToDTMotors(
                (motorPowerLF * speedCoef),
                (motorPowerLB * speedCoef),
                (motorPowerRF * speedCoef),
                (motorPowerRB * speedCoef));
    }
}
