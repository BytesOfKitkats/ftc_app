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
    private static final double DUMPER_LIFT_POWER = 0.8;
    private static final double DUMPER_DECREMENT_SPEED = 0.0008; //0.00042
    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = robot.SPEED_COEFF_FAST;
    private boolean end_game = false;

    //Intake Arm stages
    boolean up_stage_0_start = false, up_stage_0_done = false;
    boolean up_stage_1_start = false, up_stage_1_done = false;
    boolean up_stage_2_start = false;
    boolean dn_stage_0_start = false, dn_stage_0_done = false;
    boolean dn_stage_1_start = false;
    double lastArmPos = 0;
    double dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_INIT;
    boolean dumperLiftEndGame = false;

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
        double HANG_LIFT_POWER = 0.95;

        boolean hangHookRaised = false;
        boolean armUp = false;
        boolean dumpMinerals = false;

        // Initialization after Play is pressed!
        robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // A:                  Go in fast mode
            // Y:                  Go in slow mode

            moveRobot();
            moveIntakeArm(armUp);

            if (opMode.gamepad1.y) {
                speedCoef = robot.SPEED_COEFF_SLOW;
            }

            if (opMode.gamepad1.a) {
                speedCoef = robot.SPEED_COEFF_FAST;
            }

            if (opMode.gamepad1.left_bumper) {
                robot.markerServo.setPosition(robot.MARKER_SERVO_FINAL);
            }

            if (opMode.gamepad1.right_bumper) {
                robot.markerServo.setPosition(robot.MARKER_SERVO_INIT);
            }

            // GAMEPAD 2 CONTROLS
            // X:                      End Game
            // B:                      Revert from end game
            // Y                       Intake arm up
            // A                       Intake arm down
            // Dpad Up:                Intake
            // Dpad Down:              Reverse intake
            // Dpad Left:              Stop intake
            // Left Bumper:            Reset mineral dumper
            // Right Bumper:           Dump minerals
            // Left Trigger:            Manual control for lift down
            // Right Trigger:           Manual control for lift up
            // In end game
            // Dpad Up:                Raise the hanging hook
            // Dpad Down:              Lower the hanging hook

            if (opMode.gamepad2.x && !end_game) {
                end_game = true;
                Log.v("BOK", "End Game started");
                // Make sure that the intake arm is folded up
                // 1. bring the arm vertical 2. bring slides down after the arm is vertical
                if (lastArmPos > 400) {
                    armUp = true;
                    up_stage_0_start = up_stage_0_done = false;
                    up_stage_1_start = up_stage_1_done = false;
                    up_stage_2_start = false;
                }
                else {
                    armUp = false;
                    dn_stage_0_start = dn_stage_0_done = false;
                    dn_stage_1_start = false;
                }

                robot.intakeServo.setPower(0);
            }

            if (opMode.gamepad2.b && end_game) {
                end_game = false;
                dumperLiftEndGame = false;

                Log.v("BOK", "End Game reverted");
            }
            if (!end_game) {

                if (opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE ) {
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    dumpMinerals = false;
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.dumperSlideMotor.setPower(opMode.gamepad2.right_trigger);
                }
                else if (opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.dumperSlideMotor.setPower(-opMode.gamepad2.left_trigger);
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    dumpMinerals = false;
                }
                else {
                    robot.dumperSlideMotor.setPower(0);
                }

                if (opMode.gamepad2.a && armUp) {
                    dumpMinerals = false;
                    armUp = false;
                    robot.intakeArmMotorL.setTargetPositionTolerance(20);
                    robot.intakeArmMotorR.setTargetPositionTolerance(20);
                    dn_stage_0_start = dn_stage_0_done = false;
                    dn_stage_1_start = false;
                }
                else if (opMode.gamepad2.y && !armUp) {
                    armUp = true;
                    robot.intakeArmMotorL.setTargetPositionTolerance(20);
                    robot.intakeArmMotorR.setTargetPositionTolerance(20);
                    up_stage_0_start = up_stage_0_done = false;
                    up_stage_1_start = up_stage_1_done = false;
                    up_stage_2_start = false;
                }

                // Dump minerals
                if (opMode.gamepad2.left_bumper) {
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    dumpMinerals = false;
                }

                if (opMode.gamepad2.right_bumper && !dumpMinerals) {
                    dumperServoCurrPos -= 0.08;
                    dumpMinerals = true;
                    if (dumperServoCurrPos < robot.DUMPER_ROTATE_SERVO_FINAL)
                        dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_FINAL;
                    robot.dumperRotateServo.setPosition(dumperServoCurrPos);
                }
                if (dumpMinerals) {
                    dumperServoCurrPos -= 0.08;
                    if (dumperServoCurrPos < robot.DUMPER_ROTATE_SERVO_FINAL) {
                        dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_FINAL;
                        dumpMinerals = false;
                    }
                    robot.dumperRotateServo.setPosition(dumperServoCurrPos);
                }

                // Intake servo control
                if (opMode.gamepad2.dpad_up) {
                    robot.intakeServo.setPower(1);
                }
                if (opMode.gamepad2.dpad_left || opMode.gamepad2.dpad_right) {
                    robot.intakeServo.setPower(0);
                }
                if (opMode.gamepad2.dpad_down) {
                    robot.intakeServo.setPower(-1);
                }

            } // !end_game
            else {
                // End game
                // Raise or lower the hook
                if (opMode.gamepad2.dpad_up && !hangHookRaised) {
                    //robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_INIT);
                    hangHookRaised = true;
                }
                if (opMode.gamepad2.dpad_down && hangHookRaised) {
                    //robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_FINAL);
                    hangHookRaised = false;
                }
                // Hanging lift control
                if (opMode.gamepad2.left_stick_y < -GAME_TRIGGER_DEAD_ZONE) {
                    // Raise the dumper lift
                    //robot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.hangMotor.setPower(HANG_LIFT_POWER);
                    //Log.v("BOK", "Hanging Lift Pos UP " +
                    //        robot.hangMotor.getCurrentPosition());
                } else if (opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE) {
                    // Lower the lift
                    //robot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.hangMotor.setPower(-HANG_LIFT_POWER);
                    //Log.v("BOK", "Hanging Lift Pos DN " +
                    //        robot.hangMotor.getCurrentPosition());
                } else {
                    robot.hangMotor.setPower(0);
                }
            }
            robot.waitForTick(robot.WAIT_PERIOD);
        }

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRobot ()
    {
        robot.moveRobotTele(speedCoef, end_game);
    }

    private void moveIntakeArm (boolean up) {
        double currentPos = robot.intakeArmMotorR.getCurrentPosition();
        if (up) {
            if ((currentPos > 750 || up_stage_0_start) && !up_stage_0_done) {
                if (!up_stage_0_start){
                    robot.intakeArmMotorR.setTargetPosition(750);
                    robot.intakeArmMotorL.setTargetPosition(750);
                    robot.intakeArmMotorR.setPower(0.4);
                    robot.intakeArmMotorL.setPower(0.4);
                    robot.intakeServo.setPower(0);
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_FINAL+0.1);
                    dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_FINAL+0.1;
                    up_stage_0_start = true;
                }
                else if (robot.intakeArmMotorR.isBusy() && robot.intakeArmMotorL.isBusy()){
                    // adjust the dumper servo
                    double dPos = currentPos - lastArmPos;
                    dumperServoCurrPos -= (dPos * DUMPER_DECREMENT_SPEED);
                    if (dumperServoCurrPos > 0.7)
                        dumperServoCurrPos = 0.7;
                    robot.dumperRotateServo.setPosition(dumperServoCurrPos);
                }
                else {
                    up_stage_0_start = false;
                    up_stage_0_done = true;
                    Log.v("BOK", "End stage 0 " + currentPos);
                }
            } // currentPos > 750 || up_stage_0
            else if ((currentPos > 400 || up_stage_1_start) && !up_stage_1_done) {
                if (!up_stage_1_start){
                    robot.intakeArmMotorR.setTargetPosition(400);
                    robot.intakeArmMotorL.setTargetPosition(400);
                    robot.intakeArmMotorR.setPower(0.2);
                    robot.intakeArmMotorL.setPower(0.2);
                    up_stage_1_start = true;
                    Log.v("BOK", "Start stage 1 " + currentPos);
                }
                else if (robot.intakeArmMotorR.isBusy() && robot.intakeArmMotorL.isBusy()){
                    // adjust the dumper servo
                    double dPos = currentPos - lastArmPos;
                    dumperServoCurrPos -= (dPos * DUMPER_DECREMENT_SPEED);
                    if (dumperServoCurrPos > 0.7)
                        dumperServoCurrPos = 0.7;
                    robot.dumperRotateServo.setPosition(dumperServoCurrPos);
                }
                else {
                    up_stage_1_start = false;
                    up_stage_1_done = true;
                    robot.intakeServo.setPower(0); // stop the intake spinner
                    Log.v("BOK", "End stage 1 " + currentPos);
                    if (end_game) {
                        if (!dumperLiftEndGame) {
                            // Bring the slides down
                            Log.v("BOK", "End game");
                            robot.dumperSlideMotor.setTargetPosition(-6000);
                            robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.dumperSlideMotor.setPower(-DUMPER_LIFT_POWER);
                            dumperLiftEndGame = true;
                        }
                    }
                }
            } // currentPos > 400 || up_stage_1
            else if (currentPos > 250 || up_stage_2_start){
                if (!up_stage_2_start){
                    robot.intakeArmMotorL.setTargetPositionTolerance(5);
                    robot.intakeArmMotorR.setTargetPositionTolerance(5);
                    robot.intakeArmMotorR.setTargetPosition(250);
                    robot.intakeArmMotorL.setTargetPosition(250);
                    robot.intakeArmMotorR.setPower(0.06);
                    robot.intakeArmMotorL.setPower(0.06);
                    up_stage_2_start = true;
                    Log.v("BOK", "Start stage 2 " + currentPos);
                }
                else if (robot.intakeArmMotorR.isBusy() && robot.intakeArmMotorL.isBusy()){
                    // do nothing
                }
                else {
                    //up_stage_2_start = false;
                }
            } // currentPos > 250 || up_stage_2
        } // up
        else { // arm going down
            int limit = (end_game) ? 300 : 400;
            if (((currentPos < limit) || dn_stage_0_start) && !dn_stage_0_done) {
                if (!dn_stage_0_start) {
                    // no need to keep track of the dumper
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_INIT;

                    robot.intakeArmMotorR.setTargetPosition(limit);
                    robot.intakeArmMotorL.setTargetPosition(limit);
                    robot.intakeArmMotorR.setPower(0.12);
                    robot.intakeArmMotorL.setPower(0.12);
                    dn_stage_0_start = true;
                    Log.v("BOK", "Down stage 0 start " + currentPos);
                }
                else if (robot.intakeArmMotorR.isBusy() && robot.intakeArmMotorL.isBusy()){
                    // do nothing
                    //Log.v("BOK", "Curr Pos " + currentPos);
                }
                else {
                    //Log.v("BOK", "Motor not busy, going to 900");
                    if (!end_game) {
                        dn_stage_0_start = false;
                        dn_stage_0_done = true;
                        Log.v("BOK", "Down stage 0 done");
                    }
                    else {
                        if (!dumperLiftEndGame) {
                            // Bring the slides down
                            Log.v("BOK", "End game: bring lift down");
                            robot.dumperSlideMotor.setTargetPosition(-6000);
                            robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.dumperSlideMotor.setPower(-DUMPER_LIFT_POWER);
                            dumperLiftEndGame = true;
                        }
                    }
                }
            } //currentPos < 400 || down_stage_0
            else if (currentPos < 900 || dn_stage_1_start) {
                if (!dn_stage_1_start){
                    Log.v("BOK", "Entered down stage 1 " + currentPos);
                    robot.intakeArmMotorR.setTargetPosition(900);
                    robot.intakeArmMotorL.setTargetPosition(900);
                    robot.intakeArmMotorR.setPower(0.1);
                    robot.intakeArmMotorL.setPower(0.1);
                    dn_stage_1_start = true;
                }
                else if (robot.intakeArmMotorR.isBusy() && robot.intakeArmMotorL.isBusy()){
                    // adjust the dumper servo based on the current arm position if needed
                    double dPos = currentPos - lastArmPos;
                    dumperServoCurrPos -= (dPos * DUMPER_DECREMENT_SPEED);
                    if (dumperServoCurrPos < robot.DUMPER_ROTATE_SERVO_INIT)
                        dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    robot.dumperRotateServo.setPosition(dumperServoCurrPos);
                }
                else {
                    dumperServoCurrPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    robot.dumperRotateServo.setPosition(dumperServoCurrPos);
                    //dn_stage_1_start = false;
                    robot.intakeArmMotorR.setPower(0);
                    robot.intakeArmMotorL.setPower(0);
                }
            } // currentPos < 900 || down_stage_1
        }
        lastArmPos = currentPos; // record the last position
    }
}
