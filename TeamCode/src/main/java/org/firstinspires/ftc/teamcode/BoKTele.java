package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKTele
{
    // CONSTANTS
    private static final double GAME_STICK_DEAD_ZONE_LEFT_STICK = 0.3;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double INTAKE_MOTOR_CAP_SPEED = 0.9;
    private static final double Kp = 0.7, Ki = 0.525, Kd = 0.2;
    double lastErr = 0, sumErr = 0, dErrDT = 0, pid = 0, lastTime = 0, vTarget, powerSetPoint;
    int inPos, endPos, lastPos;

    private static final int INTAKE_MOTOR_LOW_LIMIT = 10;
    private static final int INTAKE_MOTOR_HIGH_LIMIT = 3250;
    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = robot.SPEED_COEFF_FAST;
    private boolean end_game = false;
    private boolean has_hanging_lift_moved = false;


    ElapsedTime intakeArmRunTime;

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
        //robot.intakeSlidesMotor.setPower(0);
        robot.dumperSlideMotor.setPower(0);
        robot.hangMotor.setPower(0);
        intakeArmRunTime = new ElapsedTime();
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
        double nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
        int currentIntakeArmPosition = 0;
        int count = 11;

        boolean runIntakeLift = true;
        boolean hangHookRaised = false;
        boolean dumperDown = false;
        boolean liftUp = false;
        boolean dump = false;
        boolean dumperTiltGold = false;
        boolean dumperTiltSilver = false;
        boolean plateTiltLow = false;
        boolean plateTiltDump = false;
        boolean intakeArmDown = false;

        int intakeArmHoldPosition = INTAKE_ARM_HIGH_POS_LIMIT;


        // Initialization after Play is pressed!
        robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_INIT);
        robot.dumperRotateServo.setPosition(robot.DUMPER_RECEIVE_SERVO);

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
                speedCoef = robot.SPEED_COEFF_SLOW;
            }
            if (opMode.gamepad1.a) {
                speedCoef = robot.SPEED_COEFF_FAST;
            }

            if (opMode.gamepad2.x && !end_game) {
                end_game = true;
                Log.v("BOK", "End Game started");
                // Make sure that the intake arm is folded up

                // Make sure that the dumper lift is down
                //robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_HANG_TILT);
                robot.dumperSlideMotor.setTargetPosition(0);
                robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER);
            }

            if (opMode.gamepad2.b && end_game) {
                end_game = false;

                Log.v("BOK", "End Game reverted");
            }


            // GAMEPAD 2 CONTROLS
            // Left Joystick:          Intake sweeper
            // Y                       Intake arm up
            // A                       Intake arm down
            // In end game, left joystick drives the hanging motor
            // Right Joystick:         Intake sweeper servo
            // Dpad Up/Down:           Switch from horizontal to vertical lift
            // Dpad Left/Right:        Switch from vertical to horizontal lift
            // Left Bumper:            Dump the minerals
            // Right Bumper:           Bring the dumper back up

            moveIntake(); // Intake sweeper

            if (!end_game) {
                if (opMode.gamepad2.dpad_up && !liftUp) {
                    count = 0;
                    dumperDown = false;
                    liftUp = true;
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setTargetPosition(robot.DUMPER_SLIDE_FINAL_POS);
                    robot.dumperSlideMotor.setPower(0.7);
                }
                if (opMode.gamepad2.dpad_down && liftUp) {
                    //robot.plateTilt.setPosition(robot.PLATE_TILT_LOW);
                    //robot.dumperTiltServo.setPosition(robot.DUMPER_TILT_SERVO_INIT);
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setTargetPosition(10);
                    robot.dumperSlideMotor.setPower(-0.7);
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    dump = false;
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
                //Log.v("BOK", "Dumper slide: " + robot.dumperSlideMotor.getCurrentPosition());
                if (opMode.gamepad2.left_bumper && !dump) {
                    dump = opMode.gamepad2.left_bumper;
                    Log.v("BOK", "LEFT bumper" + nextPos);
                    //robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_FINAL);
                }
                if (dump && (nextPos > robot.DUMPER_ROTATE_SERVO_FINAL)) {
                    nextPos -= 0.02;//0.005
                    robot.dumperRotateServo.setPosition(nextPos);
                    Log.v("BOK", "decrement" + nextPos);
                }
                if (opMode.gamepad2.right_bumper && dump) {
                    dump = false;
                    nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                }

                // Intake arm control
                if (opMode.gamepad2.a){
                    if(!robot.isRunningIntakeArmPID){
                        robot.isRunningIntakeArmPID = true;
                        intakeArmDown = true;
                        endPos = 1000;
                        vTarget = 0.5;
                        powerSetPoint = 0.5;
                        resetIntakeArmVars();
                    }
                }
                else if (opMode.gamepad2.y) {
                    if(!robot.isRunningIntakeArmPID){
                        robot.isRunningIntakeArmPID = true;
                        intakeArmDown = false;
                        endPos = 0;
                        vTarget = -0.7;
                        powerSetPoint = -0.5;
                        resetIntakeArmVars();
                    }
                }
                if (robot.isRunningIntakeArmPID && intakeArmDown){
                     if(inPos < endPos) {
                        inPos = robot.intakeArmMotor.getCurrentPosition();
                        double time = intakeArmRunTime.milliseconds();
                        double dT = time - lastTime;
                        double vEnc = (inPos - lastPos) / dT;
                        double err = vEnc - vTarget;
                        sumErr = 0.67 * sumErr + err * dT;
                        dErrDT = (err - lastErr) / dT;
                        pid = Kp * err + Ki * sumErr + Kd * dErrDT;
                        double powerApp = powerSetPoint - pid;
                        powerApp = Range.clip(powerApp, 0.0, 1.0);
                        robot.intakeArmMotor.setPower(powerApp);
                        lastErr = err;
                        lastTime = time;
                        lastPos = inPos;
                        Log.v("BOK", "Intake arm posD " + inPos + " moving at " + powerApp);
                    }
                    else {
                        robot.isRunningIntakeArmPID = false;
                        currentIntakeArmPosition = 1100;
                    }
                }
                else if (robot.isRunningIntakeArmPID && !intakeArmDown) {
                    if (inPos > endPos) {
                        inPos = robot.intakeArmMotor.getCurrentPosition();
                        double time = intakeArmRunTime.milliseconds();
                        double dT = time - lastTime;
                        double vEnc = (inPos - lastPos) / dT;
                        double err = vEnc - vTarget;
                        sumErr = 0.67 * sumErr + err * dT;
                        dErrDT = (err - lastErr) / dT;
                        pid = Kp * err + Ki * sumErr + Kd * dErrDT;
                        double powerApp = powerSetPoint - pid;
                        powerApp = Range.clip(powerApp, -1.0, 0.0);
                        robot.intakeArmMotor.setPower(powerApp);
                        lastErr = err;
                        lastTime = time;
                        lastPos = inPos;
                        Log.v("BOK", "Intake arm posU " + inPos + " moving at " + powerApp);
                    } else {
                        robot.isRunningIntakeArmPID = false;
                        currentIntakeArmPosition = -100;
                    }
                }
                else {
                    robot.intakeArmMotor.setTargetPosition(currentIntakeArmPosition);
                    robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.intakeArmMotor.setPower(0.5);
                }

                if ((count < 10) && liftUp)
                    count++;

                if ((count == 10) && liftUp) {
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setTargetPosition(1400);
                    robot.dumperSlideMotor.setPower(1.0);
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT+0.05);
                    count = 11;
                }

                if(liftUp)
                    Log.v("BOK", "Lift position " + robot.dumperSlideMotor.getCurrentPosition());

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
        robot.moveRobot(speedCoef);
    }

    private void moveIntake()
    {
        /*
         * Gamepad2: Driver 2 controls the intake servo using the left joystick for speed
         */
        // NOTE: the left joystick goes negative when pushed upwards
        double gamePad2LeftStickY = -opMode.gamepad2.left_stick_y*INTAKE_MOTOR_CAP_SPEED;
        robot.intakeMotor.setPower(gamePad2LeftStickY);
    }

    private void resetIntakeArmVars(){
        lastErr = 0; sumErr = 0; dErrDT = 0; pid = 0; lastTime = 0;
        intakeArmRunTime.reset();
        inPos = robot.intakeArmMotor.getCurrentPosition();
        lastPos = inPos;
        robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
