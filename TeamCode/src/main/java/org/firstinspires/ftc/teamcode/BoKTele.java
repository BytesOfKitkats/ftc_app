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
    //private static final double GAME_STICK_DEAD_ZONE_LEFT_STICK = 0.3;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double INTAKE_MOTOR_CAP_SPEED = 0.9;
    private static final double Kp = 0.7, Ki = 0.525, Kd = 0.2;
    double lastErr = 0, sumErr = 0, dErrDT = 0, pid = 0, lastTime = 0, vTarget, powerSetPoint;
    int inPos, endPos, lastPos;

    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = robot.SPEED_COEFF_FAST;
    private boolean end_game = false;
    private boolean isLiftingIntakeArm = false;
    private boolean hasMovedIntakeArm = false;
    private int delayCountForDumper = 0;

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
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.dumperSlideMotor.setPower(0);
        robot.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.hangMotor.setPower(0);
        intakeArmRunTime = new ElapsedTime();
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware()
    {
        // Constants
        double DUMPER_LIFT_POWER_UP = 0.95;
        double DUMPER_LIFT_POWER_HOLD = 0.1;
        double DUMPER_LIFT_POWER_DN = 0.7;
        double DUMPER_ROTATE_DECR = 0.02;
        double DUMPER_ROTATE_DECR_LOW = 0.01;
        double sorterSpeed = DUMPER_ROTATE_DECR;
        double HANG_LIFT_POWER = 0.95;

        double nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
        int currentIntakeArmPosition = 0;

        boolean hangHookRaised = true;
        boolean liftUp = false;
        boolean dump = false;
        boolean intakeArmDown = false;
        boolean isRunningIntakeArmPID = false;

        // Initialization after Play is pressed!
        robot.dumperRotateServo.setPosition(robot.DUMPER_RECEIVE_SERVO);

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // A:                  Go in fast mode
            // Y:                  Go in slow mode

            moveRobot(intakeArmDown);

            if (opMode.gamepad1.y) {
                speedCoef = robot.SPEED_COEFF_SLOW;
            }

            if (opMode.gamepad1.a) {
                speedCoef = robot.SPEED_COEFF_FAST;
            }

            if (opMode.gamepad1.b) {
                sorterSpeed = DUMPER_ROTATE_DECR_LOW;
            }

            if (opMode.gamepad1.left_bumper){
                robot.frontRotateServo.setPosition(robot.FRONT_DUMPER_SERVO_FINAL);
            }

            if (opMode.gamepad1.right_bumper){
                robot.frontRotateServo.setPosition(robot.FRONT_DUMPER_SERVO_MID);
            }

            // GAMEPAD 2 CONTROLS
            // Left Joystick:          Intake sweeper
            // In end game, left joystick drives the hanging motor
            // X:                      End Game
            // B:                      Revert from end game
            // Y                       Intake arm up
            // A                       Intake arm down
            // Right Joystick:         Intake sweeper motor
            // Dpad Up:                Take the dumper lift up
            // Dpad Down:              Bring the dumper lift down
            // Left Bumper:            Dump the minerals slowly
            // Right Bumper:           Bring the dumper back up
            // In end game
            // Dpad Up:                Raise the hanging hook
            // Dpad Down:              Lower the hanging hook

            if (opMode.gamepad2.x && !end_game) {
                end_game = true;
                Log.v("BOK", "End Game started");
                // Make sure that the intake arm is folded up

                // Make sure that the dumper lift is down
                robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                robot.dumperSlideMotor.setTargetPosition(0);
                robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER_DN);
                robot.intakeArmMotor.setTargetPosition(-100);
                robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.intakeArmMotor.setPower(0.5);
                robot.intakeMotor.setPower(0); // sweeper is off
                robot.hangMotor.setTargetPosition(150);
                robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangMotor.setPower(0.75);
            }

            if (opMode.gamepad2.b && end_game) {
                end_game = false;

                Log.v("BOK", "End Game reverted");
            }

            if (!end_game) {
                moveIntake(); // Intake sweeper

                if (opMode.gamepad2.dpad_up && !liftUp) {
                    liftUp = true;
                    currentIntakeArmPosition = 100;
                    speedCoef = robot.SPEED_COEFF_MED;
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    delayCountForDumper = 0;
                }

                if (liftUp) {
                    if (delayCountForDumper == 4) {
                        robot.dumperSlideMotor.setTargetPosition(robot.DUMPER_SLIDE_FINAL_POS);
                        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER_UP);
                    } else
                        delayCountForDumper++;
                }

                if (opMode.gamepad2.dpad_down && liftUp) {
                    //robot.plateTilt.setPosition(robot.PLATE_TILT_LOW);
                    //robot.dumperTiltServo.setPosition(robot.DUMPER_TILT_SERVO_INIT);
                    speedCoef = robot.SPEED_COEFF_FAST;
                    robot.dumperSlideMotor.setTargetPosition(10);
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER_DN);
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    dump = false;
                    liftUp = false;
                    sorterSpeed = DUMPER_ROTATE_DECR;
                }
                else { // Neither Dpad Up or Dpad Down is pressed
                    // Hold the lift's last position, but there is a minimum so that the
                    // string remains tight.
                    if (liftUp && !robot.dumperSlideMotor.isBusy()) {
                        robot.dumperSlideMotor.setPower(0);
                        robot.dumperSlideMotor.setTargetPosition(robot.DUMPER_SLIDE_FINAL_POS);
                        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER_HOLD);
                    }
                }

                // Dump minerals
                if (opMode.gamepad2.left_bumper && !dump) {
                    dump = opMode.gamepad2.left_bumper;
                    //Log.v("BOK", "LEFT bumper" + nextPos);
                }
                if (dump && (nextPos > robot.DUMPER_ROTATE_SERVO_FINAL)) {
                    nextPos -= sorterSpeed;
                    robot.dumperRotateServo.setPosition(nextPos);
                    //Log.v("BOK", "decrement" + nextPos);
                }
                if (opMode.gamepad2.right_bumper && dump) {
                    dump = false;
                    sorterSpeed = DUMPER_ROTATE_DECR;
                    nextPos = robot.DUMPER_RECEIVE_SERVO;
                    robot.dumperRotateServo.setPosition(robot.DUMPER_RECEIVE_SERVO);
                }

                // Intake arm control
                // TODO: add the ability to cancel the arm movement
                if (opMode.gamepad2.a && !isLiftingIntakeArm){ // bring the arm down
                    //Log.v("BOK", "A pressed at " + robot.intakeArmMotor.getCurrentPosition());
                    if (isRunningIntakeArmPID && !intakeArmDown) {
                        // cancel the up movement
                        isRunningIntakeArmPID = false;
                        robot.intakeMotor.setPower(-1);
                    }
                    if(!isRunningIntakeArmPID && !intakeArmDown) {
                        isRunningIntakeArmPID = true;
                        intakeArmDown = true;
                        endPos = 1000;
                        vTarget = 0.5;
                        powerSetPoint = 0.5;
                        resetIntakeArmVars();
                    }
                }
                else if (opMode.gamepad2.y && !isLiftingIntakeArm) { // bring the arm up
                    //Log.v("BOK", "Y pressed at " + robot.intakeArmMotor.getCurrentPosition());
                    robot.dumperRotateServo.setPosition(robot.DUMPER_RECEIVE_SERVO);
                    if (isRunningIntakeArmPID && intakeArmDown) {
                        // cancel the down movement
                        isRunningIntakeArmPID = false;
                    }
                    if(!isRunningIntakeArmPID && intakeArmDown){
                        isRunningIntakeArmPID = true;
                        intakeArmDown = false;
                        endPos = 0;
                        vTarget = -0.7;
                        powerSetPoint = -0.7;
                        resetIntakeArmVars();
                    }
                }
                if (isRunningIntakeArmPID && intakeArmDown){
                     if (inPos < endPos) {
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
                        //Log.v("BOK", "Intake arm posD " + inPos + " moving at " + powerApp);
                    } // if (inPos < endPos)
                    else {
                        isRunningIntakeArmPID = false;
                        currentIntakeArmPosition = 1100;
                    }
                } // if (isRunningIntakeArmPID && intakeArmDown)
                else if (isRunningIntakeArmPID && !intakeArmDown) {
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
                        //Log.v("BOK", "Intake arm posU " + inPos + " moving at " + powerApp);
                    } // if (inPos > endPos)
                    else {
                        isRunningIntakeArmPID = false;
                        currentIntakeArmPosition = -100;
                    }
                } // else if (isRunningIntakeArmPID && !intakeArmDown)
                else {
                    //Log.v("BOK", "in ELSE isLiftingIntakeArm " + isLiftingIntakeArm + " hasMovedIntakeArm " + hasMovedIntakeArm);
                    if (!isLiftingIntakeArm && hasMovedIntakeArm) {
                        //Log.v ("BOK", "Holding at " + currentIntakeArmPosition);
                        robot.intakeArmMotor.setTargetPosition(currentIntakeArmPosition);
                        robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.intakeArmMotor.setPower(0.5);
                    }
                }

                //the arm is down on the floor but not extended
                if ((opMode.gamepad2.right_stick_y > GAME_TRIGGER_DEAD_ZONE)
                        && !hasMovedIntakeArm) {
                    robot.intakeArmMotor.setPower(0.5*opMode.gamepad2.right_stick_y);

                }

                // use right joystick to move the arm up
                else if ((opMode.gamepad2.right_stick_y < -GAME_TRIGGER_DEAD_ZONE)
                        && !hasMovedIntakeArm) {
                    robot.intakeArmMotor.setPower(0.4*opMode.gamepad2.right_stick_y);

                }

                if ((opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE)
                        && !isLiftingIntakeArm) {
                    isLiftingIntakeArm = true;
                    robot.intakeArmMotor.setPower(-0.6);
                    Log.v("BOK", "Tele: Started lifting Intake ARM");
                }

                if ((opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE)
                        && isLiftingIntakeArm) {
                    hasMovedIntakeArm = true;
                    isLiftingIntakeArm = false;
                    robot.intakeArmMotor.setPower(0);
                    robot.intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Log.v("BOK", "Tele: Initialized Intake ARM");
                }
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
                    // Raise the dumper lift
                    robot.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.hangMotor.setPower(HANG_LIFT_POWER);
                    Log.v("BOK", "Hanging Lift Pos UP " +
                            robot.hangMotor.getCurrentPosition());
                } else if (opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE) {
                    // Lower the lift
                    robot.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.hangMotor.setPower(-HANG_LIFT_POWER);
                    Log.v("BOK", "Hanging Lift Pos DN " +
                            robot.hangMotor.getCurrentPosition());
                } else if (robot.hangMotor.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                    robot.hangMotor.setPower(0);
                }
                if (hangHookRaised){
                    robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_INIT);
                }
            }

            robot.waitForTick(robot.WAIT_PERIOD);
        }

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRobot(boolean intakeArmDown)
    {
        robot.moveRobotTele(speedCoef, intakeArmDown);
    }

    private void moveIntake()
    {
        /*
         * Gamepad2: Driver 2 controls the intake motor using the left joystick for speed
         */
        // NOTE: the left joystick goes negative when pushed upwards
        double gamePad2LeftStickY = -opMode.gamepad2.left_stick_y*INTAKE_MOTOR_CAP_SPEED;
        gamePad2LeftStickY *= 0.8; // TODO: cubic control??
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
