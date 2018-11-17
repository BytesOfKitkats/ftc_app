package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKTele
{
    // CONSTANTS
    private static final double GAME_STICK_DEAD_ZONE = 0.1;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double SPEED_COEFF_SLOW = 0.35;
    private static final double SPEED_COEFF_FAST = 0.8;
    private static final double SPEED_COEFF_TURN = 0.7;

    private static final int INTAKE_MOTOR_LOW_LIMIT = 10;
    private static final int INTAKE_MOTOR_HIGH_LIMIT = 3250;
    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = SPEED_COEFF_FAST;
    private boolean end_game = false;
    private boolean has_hanging_lift_moved = false;

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
        //robot.intakeArmMotor.setPower(0);
        robot.intakeSlidesMotor.setPower(0);
        robot.dumperSlideMotor.setPower(0);
        robot.hangMotor.setPower(0);
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware()
    {
        // Constants
        double INTAKE_ARM_POWER_LOW = 0.1;
        double INTAKE_ARM_POWER_NORMAL = 0.5;
        double INTAKE_ARM_POWER_HIGH = 0.6;
        int INTAKE_ARM_HIGH_POS_SLOW = 250;
        int INTAKE_ARM_HIGH_POS_LIMIT = 0;
        int INTAKE_ARM_LOW_POS_SLOW = 360;
        int INTAKE_ARM_LOW_POS_LIMIT = 460;
        double INTAKE_LIFT_POWER = 0.9;
        int INTAKE_LIFT_LOW_POS = 0;
        double DUMPER_LIFT_POWER = 0.5;
        int DUMPER_LIFT_LOW_POS = 0;
        double HANG_LIFT_POWER = 0.5;
        int HANG_LIFT_LOW_POS = 0;
        double COUNTS_PER_MOTOR_REV_AM60 = 1680;
        double DEGREES_ARM_MOTOR_MOVES = 172;

        boolean runIntakeLift = true;
        boolean hangHookRaised = false;
        boolean dumperDown = false;
        boolean liftUp = false;
        boolean dumperTiltGold = false;
        boolean dumperTiltSilver = false;
        int intakeArmHoldPosition = INTAKE_ARM_HIGH_POS_LIMIT;

        // Initialization after Play is pressed!
        robot.samplerLeftServo.setPosition(robot.SAMPLER_LEFT_SERVO_INIT);
        robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_INIT);
        robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_INIT);
        robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_ANGLE);

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // X:                  End Game
            // B:                  Revert from end game
            // A:                  Go in fast mode
            // Y:                  Go in slow mode
            // Dpad Up:            Raise the hanging hook
            // Dpad Dn:            Lower the hanging hook
            moveRobot();

            if (opMode.gamepad1.y) {
                speedCoef = SPEED_COEFF_SLOW;
            }
            if (opMode.gamepad1.a) {
                speedCoef = SPEED_COEFF_FAST;
            }

            if (opMode.gamepad2.x && !end_game) {
                end_game = true;
                Log.v("BOK", "End Game started");
                // Make sure that the intake arm is folded up

                // Make sure that the dumper lift is down
                robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_HANG_TILT);
                robot.dumperSlideMotor.setTargetPosition(0);
                robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER);
            }

            if (opMode.gamepad2.b && end_game) {
                end_game = false;

                Log.v("BOK", "End Game reverted");
            }


            // GAMEPAD 2 CONTROLS
            // Left Joystick:          Vertical (dumper) & horizontal (intake) lift
            // In end game, left joystick drives the hanging motor
            // Right Joystick:         Intake sweeper servo
            // Dpad Up/Down:           Switch from horizontal to vertical lift
            // Dpad Left/Right:        Switch from vertical to horizontal lift
            // Left Bumper:            Dump the minerals
            // Right Bumper:           Bring the dumper back up

            moveIntakeServo(); // Intake sweeper

            if (!end_game) {
                if (opMode.gamepad2.dpad_up && !liftUp) {
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setTargetPosition(7025);
                    robot.dumperSlideMotor.setPower(1.0);
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT+0.1);
                    dumperDown = false;
                    liftUp=true;
                }
                if (opMode.gamepad2.dpad_down && liftUp) {
                    robot.dumperTiltServo.setPosition(robot.DUMPER_TILT_SERVO_INIT);
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setTargetPosition(10);
                    robot.dumperSlideMotor.setPower(0.7);
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    dumperDown = false;
                    liftUp = false;
                }
                else {
                    // Hold the lift's last position, but there is a minimum so that the
                    // string remains tight.
                    if(!robot.dumperSlideMotor.isBusy()) {
                        robot.dumperSlideMotor.setPower(0);
                        int currentLiftPosition = robot.dumperSlideMotor.getCurrentPosition();
                        if (currentLiftPosition < DUMPER_LIFT_LOW_POS) {
                            //Log.v("BOK", "Dumper lift check: " +
                            //        robot.dumperSlideMotor.getCurrentPosition());
                            currentLiftPosition = DUMPER_LIFT_LOW_POS;
                        }
                        robot.dumperSlideMotor.setTargetPosition(currentLiftPosition);
                        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER);
                    }
                }
                if (opMode.gamepad2.left_bumper && !dumperDown &&
                        (robot.dumperSlideMotor.getCurrentPosition() > 3000)) {
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_FINAL);
                    dumperDown = true;
                }
                if (opMode.gamepad2.right_bumper && dumperDown) {
                    robot.dumperTiltServo.setPosition(robot.DUMPER_TILT_SERVO_INIT);
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    dumperDown = false;
                }

                // Intake arm control
                int currentIntakeArmPosition = robot.intakeArmMotor.getCurrentPosition();
                double intakeArmPower = INTAKE_ARM_POWER_NORMAL;
                if ((currentIntakeArmPosition >= INTAKE_ARM_LOW_POS_SLOW) ||
                        (currentIntakeArmPosition <= INTAKE_ARM_HIGH_POS_SLOW))
                    intakeArmPower = INTAKE_ARM_POWER_LOW;
                if (opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
                    // Lower the intake arm
                    //if (robot.intakeArmMotor.getCurrentPosition() < INTAKE_ARM_LOW_POS_LIMIT) {
                        robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.intakeArmMotor.setPower(intakeArmPower);
                        //Log.v("BOK", "Intake Arm Pos low " + currentIntakeArmPosition +
                        //        " at power: " + intakeArmPower);
                        intakeArmHoldPosition = currentIntakeArmPosition;
                    //}
                    /*else {
                        intakeArmHoldPosition = 0;
                        Log.v("BOK", "Setting low limit");
                    }*/
                } else if (opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE) {
                    if(!has_hanging_lift_moved)
                        robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_ANGLE);
                    if (robot.intakeArmMotor.getCurrentPosition() > INTAKE_ARM_HIGH_POS_LIMIT) {

                        if (robot.intakeArmMotor.getCurrentPosition() < INTAKE_ARM_HIGH_POS_SLOW)
                            intakeArmPower = INTAKE_ARM_POWER_LOW;
                        else
                            intakeArmPower = INTAKE_ARM_POWER_HIGH;

                        // Raise the intake arm
                        robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.intakeArmMotor.setPower(-intakeArmPower);
                        intakeArmHoldPosition = currentIntakeArmPosition;
                        //Log.v("BOK", "Intake Arm Pos high " + currentIntakeArmPosition +
                        //        " at power: " + intakeArmPower);
                    }
                    else
                        intakeArmHoldPosition = INTAKE_ARM_HIGH_POS_LIMIT;
                } else {
                    // Hold the lift's last position, but there is a minimum & maximum
                    intakeArmPower = 0;
                    robot.intakeArmMotor.setPower(intakeArmPower);
                    if(intakeArmHoldPosition != 0) {

                        intakeArmPower = INTAKE_ARM_POWER_NORMAL;
                        //Log.v("BOK", "Intake Arm Pos hold " + intakeArmHoldPosition +
                        //        " at power: " + intakeArmPower);
                        //int diff = Math.abs(intakeArmHoldPosition - INTAKE_ARM_LOW_POS_LIMIT);
                        //if (diff > 10) {
                            robot.intakeArmMotor.setTargetPosition(intakeArmHoldPosition);
                            robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.intakeArmMotor.setPower(intakeArmPower);
                        //}

                    }
                   // Log.v("BOK", "Intake Motor PWR: " + intakeArmPower);
                }
                if (runIntakeLift) {
                    int currentIntakeLiftPos = robot.intakeSlidesMotor.getCurrentPosition();
                    // Intake lift control
                    if ((opMode.gamepad2.left_stick_x > GAME_TRIGGER_DEAD_ZONE) &&
                            (currentIntakeLiftPos < INTAKE_MOTOR_HIGH_LIMIT)){

                        // Extend the intake arm
                        robot.intakeSlidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.intakeSlidesMotor.setPower(INTAKE_LIFT_POWER);
                        //Log.v("BOK", "Intake Lift Pos + " +
                        //        robot.intakeSlidesMotor.getCurrentPosition());
                    } else if ((opMode.gamepad2.left_stick_x < -GAME_TRIGGER_DEAD_ZONE) &&
                            (currentIntakeLiftPos > INTAKE_MOTOR_LOW_LIMIT)){
                       // Retract the intake arm
                        //Log.v("BOK", "Intake Lift Pos - " +
                        //        robot.intakeSlidesMotor.getCurrentPosition());
                        robot.intakeSlidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.intakeSlidesMotor.setPower(-INTAKE_LIFT_POWER);
                    } else {
                        // Hold the lift's last position, but there is a minimum so that the
                        // string remains tight.
                        robot.intakeSlidesMotor.setPower(0);
                        int currentLiftPosition = robot.intakeSlidesMotor.getCurrentPosition();
                        if (currentLiftPosition < INTAKE_LIFT_LOW_POS) {
                            //Log.v("BOK", "Intake lift check: " +
                            //        robot.intakeSlidesMotor.getCurrentPosition());
                            //currentLiftPosition = INTAKE_LIFT_LOW_POS;
                        }
                        robot.intakeSlidesMotor.setTargetPosition(currentLiftPosition);
                        robot.intakeSlidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.intakeSlidesMotor.setPower(INTAKE_LIFT_POWER);
                    }

                }
                if (opMode.gamepad2.y && !dumperTiltGold &&
                        (robot.dumperSlideMotor.getCurrentPosition() > 6000)){
                    robot.dumperTiltServo.setPosition(robot.DUMPER_TILT_SERVO_GOLD);
                }
                dumperTiltGold = opMode.gamepad2.y;
                if (opMode.gamepad2.a && !dumperTiltSilver &&
                        (robot.dumperSlideMotor.getCurrentPosition() > 6000)){
                    robot.dumperTiltServo.setPosition(robot.DUMPER_TILT_SERVO_INIT);
                }
                dumperTiltSilver = opMode.gamepad2.a;
                /*else { // !runIntakeLift
                    // Dumper lift control
                    if ((opMode.gamepad2.left_stick_y < -GAME_TRIGGER_DEAD_ZONE)
                    &&(robot.dumperSlideMotor.getCurrentPosition() < 3400)){
                        // Raise the dumper lift
                        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER);
                        Log.v("BOK", "Dumper Lift Pos " +
                                robot.dumperSlideMotor.getCurrentPosition());
                    } else if (opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE) {

                        // Make sure that the dumper is back in init
                        robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                        dumperDown = false;
                        //if (robot.dumperSlideMotor.getCurrentPosition() > 0) {
                            // Lower the lift
                            robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            robot.dumperSlideMotor.setPower(-DUMPER_LIFT_POWER);
                        //}

                    } else {
                        // Hold the lift's last position, but there is a minimum so that the
                        // string remains tight.
                        robot.dumperSlideMotor.setPower(0);
                        int currentLiftPosition = robot.dumperSlideMotor.getCurrentPosition();
                        if (currentLiftPosition < DUMPER_LIFT_LOW_POS) {
                            Log.v("BOK", "Dumper lift check: " +
                                    robot.dumperSlideMotor.getCurrentPosition());
                            currentLiftPosition = DUMPER_LIFT_LOW_POS;
                        }
                        robot.dumperSlideMotor.setTargetPosition(currentLiftPosition);
                        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER);
                    }
                } // !runIntakeLift*/
            } // !end_game
            else {
                // End game
                // Raise or lower the hook
                if (opMode.gamepad2.dpad_up && !hangHookRaised) {
                    robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_INIT);
                    hangHookRaised = true;
                }
                if (opMode.gamepad2.dpad_down && hangHookRaised) {
                    robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_FINAL);
                    hangHookRaised = false;
                }
                // Hanging lift control
                if (opMode.gamepad2.left_stick_y < -GAME_TRIGGER_DEAD_ZONE) {
                    has_hanging_lift_moved = true;
                    // Raise the dumper lift
                    robot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.hangMotor.setPower(HANG_LIFT_POWER);
                    //Log.v("BOK", "Hanging Lift Pos UP " +
                    //        robot.hangMotor.getCurrentPosition());
                } else if (opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE) {
                    has_hanging_lift_moved = true;
                    //if (robot.hangMotor.getCurrentPosition() > 0) {
                        // Lower the lift
                        robot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.hangMotor.setPower(-HANG_LIFT_POWER);
                    //Log.v("BOK", "Hanging Lift Pos DN " +
                    //        robot.hangMotor.getCurrentPosition());
                    //}

                } else {
                    // Hold the lift's last position, but there is a minimum so that the
                    // string remains tight.
                    robot.hangMotor.setPower(0);
                    /*
                    int currentLiftPosition = robot.hangMotor.getCurrentPosition();
                    if (currentLiftPosition < HANG_LIFT_LOW_POS) {
                        Log.v("BOK", "Hang lift check: " +
                                robot.hangMotor.getCurrentPosition());
                        currentLiftPosition = HANG_LIFT_LOW_POS;
                    }
                    robot.hangMotor.setTargetPosition(currentLiftPosition);
                    robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.hangMotor.setPower(DUMPER_LIFT_POWER);
                */}
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
        //double gamePad1LeftStickX = opMode.gamepad1.left_stick_x;
        double gamePad1RightStickX = opMode.gamepad1.right_stick_x;

        if (speedCoef == SPEED_COEFF_FAST) {
            //gamePad1LeftStickX = Math.pow(gamePad1LeftStickX, 3);
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

        if ((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE))
        {
            motorPowerLF = -gamePad1LeftStickY;
            motorPowerLB = -gamePad1LeftStickY;
            motorPowerRF = -gamePad1LeftStickY;
            motorPowerRB = -gamePad1LeftStickY;
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
            motorPowerRF = -gamePad1RightStickX;
            motorPowerRB = -gamePad1RightStickX;

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

    private void moveIntakeServo()
    {
        double servoPower = 0.0;
        /*
         * Gamepad2: Driver 2 controls the intake servo using the left joystick for speed
         */
        // NOTE: the left joystick goes negative when pushed upwards
        double gamePad2RightStickY = opMode.gamepad2.right_stick_y;
        if ((Math.abs(gamePad2RightStickY) > GAME_STICK_DEAD_ZONE) ||
            (Math.abs(gamePad2RightStickY) < -GAME_STICK_DEAD_ZONE)) {
            servoPower = -gamePad2RightStickY;
            //Log.v("BOK","Intake Servo:" + String.format("%.2f", servoPower));
        }
        robot.intakeSweeperServo.setPower(servoPower);
    }
}
