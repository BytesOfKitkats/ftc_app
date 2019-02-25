package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.vuforia.CameraDevice;

public class BoKAutoTest extends BoKAutoCommon {

    // Constructor
    public BoKAutoTest() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        boolean[] arrayTests = {
                false, // 4 DT motors
                false,  // Dumper lift motor & servo
                false, // Intake arm motor & intake servo
                false, // Hanging lift motor
                false,  // marker servo
                false,  // distance sensor & servo
                false,  // autonomous test
                true}; // take picture


        if (arrayTests[0]) {
            // move (test the 4 DC motors
            opMode.telemetry.addData("Test: ", "Moving forward");
            opMode.telemetry.update();
            robot.testDTMotors();  //move(0.5, 0.5, 12, true, 6);
            opMode.telemetry.addData("Test: ", "Moving forward complete");
            opMode.telemetry.update();
            opMode.sleep(2000);
        } // if (arrayTests[0]) : test DC Motors

        while (opMode.opModeIsActive() && arrayTests[1] && !opMode.gamepad1.a) {
        }

        if (arrayTests[1] && opMode.opModeIsActive()) {
            // test Dumper lift motor
            opMode.telemetry.addData("Test: ", "Moving dumper lift");
            opMode.telemetry.update();
            robot.dumperSlideMotor.setTargetPosition(robot.DUMPER_SLIDE_FINAL_POS);
            robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.dumperSlideMotor.setPower(0.8);

            while (opMode.opModeIsActive() && robot.dumperSlideMotor.isBusy()) {
                // do nothing
            }

            while (opMode.opModeIsActive() && !opMode.gamepad1.x) {
                boolean dump = false;
                if (opMode.gamepad1.a) {

                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_FINAL);
                }
                if (opMode.gamepad1.b) {

                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                }

            }

            robot.dumperSlideMotor.setTargetPosition(0);
            robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.dumperSlideMotor.setPower(-0.7);
            while (opMode.opModeIsActive() && robot.dumperSlideMotor.isBusy()) {
                // do nothing
            }
            //Log.v("BOK", "Dumper slide done: " + robot.dumperSlideMotor.getCurrentPosition());
            opMode.telemetry.addData("Test: ", "Moving dumper lift complete");
            opMode.telemetry.update();
        } // if (arrayTests[1]) : dumper lift motor and servo


        while (opMode.opModeIsActive() && arrayTests[2] && !opMode.gamepad1.a) {
        }

        if (arrayTests[2] && opMode.opModeIsActive()) {
            // Test intake arm motor and intake motor
            opMode.telemetry.addData("Test: ", "Intake arm motor");
            opMode.telemetry.update();
            moveIntakeArmPID(1000/*enc count*/, 0.5/*power*/, 0.5/*vTarget*/, 3/*seconds*/);
            // Complete the final position of the intake arm
            robot.intakeArmMotorL.setTargetPosition(1200);
            robot.intakeArmMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeArmMotorL.setPower(0.5);
            robot.intakeArmMotorL.setPower(0);
            robot.intakeServo.setPower(1);
            opMode.sleep(1000);
            robot.intakeServo.setPower(0);
            opMode.telemetry.addData("Test: ", "Intake arm complete");
            opMode.telemetry.update();
        }

        while (opMode.opModeIsActive() && arrayTests[3] && !opMode.gamepad1.a) {
        }

        if (arrayTests[3] && opMode.opModeIsActive()) {
            // Test hanging lift
            opMode.telemetry.addData("Test: ", "Hanging arm motor");
            opMode.telemetry.update();

            robot.hangMotor.setTargetPosition(robot.HANG_LIFT_HIGH_POS);
            robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangMotor.setPower(0.5);
            while (opMode.opModeIsActive() && robot.hangMotor.isBusy()) {

            }
            robot.hangMotor.setPower(0);
            opMode.telemetry.addData("Test: ", "Moving hanging complete");
            opMode.telemetry.update();
        }

        while (opMode.opModeIsActive() && arrayTests[4] && !opMode.gamepad1.a) {
        }

        if (arrayTests[4] && opMode.opModeIsActive()) {
            // Test marker servo
            opMode.telemetry.addData("Test: ", "Marker servo");
            opMode.telemetry.update();
            dumpMarker();
        }

        while (opMode.opModeIsActive() && arrayTests[5] && !opMode.gamepad1.a) {
        }

        if (arrayTests[5] && opMode.opModeIsActive()) {
            // Test sensor servo & sensor
            opMode.telemetry.addData("Test: ", "Distance sensor");
            opMode.telemetry.update();
            robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_FINAL);
            opMode.sleep(500);

            while (opMode.opModeIsActive() && !opMode.gamepad1.x) {
                double dist = robot.getDistanceCM(robot.distanceBack, 150, 2);
                opMode.telemetry.addData("Distance: ", dist);
                opMode.telemetry.update();
            }
            opMode.sleep(500);

            robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_INIT);
            opMode.sleep(1000);

        }
        opMode.telemetry.addData("Test: ", "All tests complete");
        opMode.telemetry.update();

        while (opMode.opModeIsActive() && arrayTests[6] && !opMode.gamepad1.a) {
        }

        if (arrayTests[6] && opMode.opModeIsActive()) {
            // Autonomous tests
            //robot.moveIntakeArmPID(400, 0.25, 0.2, 10);
            //gyroTurn(DT_TURN_SPEED_LOW, 0, 45, DT_TURN_THRESHOLD_LOW, false, false, 4);

            //strafe(0.5, 3, false, 6);
            //dropIntakeArmAndExtend();
            //followHeadingPIDBack(0, -0.3, 30, false, 6);
            followHeadingPID(0, 0.5, 35, 45, true, 5);
        }

        while (opMode.opModeIsActive() && arrayTests[7] && !opMode.gamepad1.a) {
        }

        if (arrayTests[7] && opMode.opModeIsActive()) {

            // CameraDevice.getInstance().setFlashTorchMode(true);

            int numPics = 0;
            boolean x = false;
            while (opMode.opModeIsActive()) {
                if (opMode.gamepad1.x && !x) {
                    takePicture("SamplingImage_" + numPics + ".png");
                    numPics++;
                    opMode.telemetry.addData("Picture: ", "Taking picture " + numPics);
                    opMode.telemetry.update();

                }
                x = opMode.gamepad1.x;
            }
        } // arrayTests[7]: take picture
    }
}
