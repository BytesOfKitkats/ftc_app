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

    private BoKHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = robot.SPEED_COEFF_FAST;
    private double turnCoef = robot.SPEED_COEFF_TURN;
    private boolean end_game = false;
    private double holdIntakeSpeed = 0;
    private boolean isHoldingSpeed = false;

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

    public BoKTeleStatus runSoftware(boolean atCrater)
    {
        // Constants
        double DUMPER_LIFT_POWER_UP = 0.95;
        int DUMPER_LIFT_MID_POS = robot.DUMPER_SLIDE_FINAL_POS/3;
        double DUMPER_LIFT_POWER_HOLD = 0.1;
        double DUMPER_LIFT_POWER_DN = 0.5;
        double HANG_LIFT_POWER = 0.95;

        boolean liftUp = false;
        boolean dump = false;
        boolean liftSlow = false;
        boolean liftUpInit = false;
        // Initialize intake box to up
        boolean gateOpen = false;
        boolean bPressed = false, yPressed = false, aPressed = false;
        int dumperGateCount = 0;

        // Initialization after Play is pressed!
        robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_INIT);
        robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_UP);
        robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_UP);
        robot.intakeGateServo.setPosition(robot.INTAKE_GATE_SERVO_CLOSED);
        if (!atCrater)
            turnCoef *= 2;

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // A:                  Go in fast mode
            // Y:                  Go in slow mode

            moveRobot();

            if (opMode.gamepad1.y) {
                speedCoef = robot.SPEED_COEFF_SLOW;
            }

            if (opMode.gamepad1.a) {
                speedCoef = robot.SPEED_COEFF_FAST;
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
                robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_INIT);
                robot.dumperSlideMotor.setTargetPosition(250);
                robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER_DN);
                robot.intakeSlideMotor.setTargetPosition(150 /*TODO change*/);
                robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.intakeSlideMotor.setPower(0.5);
                robot.intakeMotor.setPower(0); // sweeper is off
                robot.intakeLeftServo.setPosition(0.5);
                robot.intakeRightServo.setPosition(0.5);
            }

            if (opMode.gamepad2.b && end_game) {
                end_game = false;
                robot.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.hangMotor.setPower(0);

                Log.v("BOK", "End Game reverted");
            }

            if (!end_game) {
                moveIntake(); // Intake sweeper

                if (opMode.gamepad2.dpad_up && !liftUp) {
                    liftUp = true;
                    liftUpInit = true;
                    dump = false;
                    speedCoef = robot.SPEED_COEFF_MED;
                    robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_INIT);
                    robot.intakeMotor.setPower(0);
                    robot.dumperSlideMotor.setTargetPosition(robot.DUMPER_SLIDE_FINAL_POS);
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER_UP);
                }

                else if (opMode.gamepad2.dpad_down && liftUp) {
                    speedCoef = robot.SPEED_COEFF_FAST;
                    robot.dumperSlideMotor.setTargetPosition(0);
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER_DN);
                    robot.intakeGateServo.setPosition(robot.INTAKE_GATE_SERVO_CLOSED);
                    dump = false;
                    liftUp = false;
                    liftSlow = false;
                }

                else { // Neither Dpad Up or Dpad Down is pressed
                    // Hold the lift's last position, but only when the lift is up
                    if (liftUp && !robot.dumperSlideMotor.isBusy()) {
                        robot.dumperSlideMotor.setPower(0);
                        robot.dumperSlideMotor.setTargetPosition(robot.DUMPER_SLIDE_FINAL_POS);
                        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER_HOLD);
                    }
                    if (!liftUp) {
                        if ((robot.dumperSlideMotor.getCurrentPosition() < 900) && gateOpen) {
                            robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_INIT);
                            gateOpen = false;
                        }
                        if (!liftSlow && liftUpInit &&
                                (robot.dumperSlideMotor.getCurrentPosition() < DUMPER_LIFT_MID_POS)) {
                            robot.dumperSlideMotor.setPower(0.5);
                            liftSlow = true;
                        }
                    }
                }

                // Dump minerals
                if (opMode.gamepad2.left_bumper && !dump) {
                    robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_FINAL);
                    dump = true;
                    gateOpen = true;
                }

                if ((opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE) && dump) {
                    robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_INIT-0.1);
                    gateOpen = false;
                    dumperGateCount = 1;
                }

                if (dump && (dumperGateCount > 0)) {
                    if (++ dumperGateCount > 5) {
                        dumperGateCount  = 0;
                        robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_FINAL);
                        gateOpen = true;
                    }
                }

                // Intake box control
                if (opMode.gamepad2.a /*&& !intakeBoxUp*/ && !aPressed) {
                    robot.intakeGateServo.setPosition(robot.INTAKE_GATE_SERVO_CLOSED);
                    robot.intakeRightServo.setPosition(0.15);
                    robot.intakeLeftServo.setPosition(0.85);
                    aPressed = true;
                    yPressed = false;
                    bPressed = false;
                }

                if (opMode.gamepad2.y && !yPressed) {
                    robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_UP);
                    robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_UP);
                    //intakeBoxUp = true;
                    yPressed = true;
                    aPressed = false;
                    bPressed = false;
                }

                if (opMode.gamepad2.b && !bPressed) {
                    robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_MID);
                    robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_MID);
                    //intakeBoxUp = false;
                    bPressed = true;
                    yPressed = false;
                    aPressed = false;
                }

                // Intake gate control
                if (opMode.gamepad2.dpad_left) {
                    robot.intakeGateServo.setPosition(robot.INTAKE_GATE_SERVO_OPEN);
                }

                // Intake arm control
                if(opMode.gamepad2.left_stick_y > 0.2){
                    // Go back slowly
                    robot.intakeSlideMotor.setPower(opMode.gamepad2.left_stick_y*0.7);
                }
                else if(opMode.gamepad2.left_stick_y < -0.2){
                    // Go forward
                    robot.intakeSlideMotor.setPower(opMode.gamepad2.left_stick_y);
                    Log.v("BOK", "Slide enc " + robot.intakeSlideMotor.getCurrentPosition());
                }
                else if (opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE) {
                    robot.intakeSlideMotor.setTargetPosition(-10/*TODO change*/);
                    robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.intakeSlideMotor.setPower(0.7);
                    robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_UP);
                    robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_UP);
                }
                else if (!robot.intakeSlideMotor.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION))
                    robot.intakeSlideMotor.setPower(0);
                else if (!robot.intakeSlideMotor.isBusy()) {
                    robot.intakeSlideMotor.setPower(0);
                    robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            } // !end_game
            else {
                // End game
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
                } else {
                    robot.hangMotor.setPower(0);
                }
            }

            robot.waitForTick(robot.WAIT_PERIOD);
        }

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRobot()
    {
        robot.moveRobotTele(speedCoef, turnCoef, end_game);
    }

    private void moveIntake()
    {
        /*
         * Gamepad2: Driver 2 controls the intake motor using the right joystick for speed
         */
        // NOTE: the right joystick goes negative when pushed upwards
        double gamePad2RightStickY = -opMode.gamepad2.right_stick_y;
        if (opMode.gamepad2.right_stick_button)
            isHoldingSpeed = false;
        if (opMode.gamepad2.right_bumper && !isHoldingSpeed) {
            holdIntakeSpeed = gamePad2RightStickY;
            isHoldingSpeed = true;
        }
        else if ((Math.abs(gamePad2RightStickY) < GAME_TRIGGER_DEAD_ZONE) && !isHoldingSpeed) {
            robot.intakeMotor.setPower(0);
        }
        else {
            if (isHoldingSpeed)
                robot.intakeMotor.setPower(holdIntakeSpeed);
            else
                robot.intakeMotor.setPower(gamePad2RightStickY);
        }
    }
}
