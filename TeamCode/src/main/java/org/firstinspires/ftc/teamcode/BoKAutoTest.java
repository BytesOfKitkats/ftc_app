package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class BoKAutoTest extends BoKAutoCommon {

    // Constructor
    public BoKAutoTest() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        boolean[] arrayTests = {
                false, // 4 DT motors
                false,  // Intake arm motor & intake servo
                false,  // Dumper lift motor & servo; MUST RUN INTAKE ARM TEST FIRST
                false,  // Hanging lift motor
                false,  // marker servo
                false,  // distance sensor & servo
                true,  // autonomous test
                false, // take picture
                false}; // dumper servo test


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
            // Test intake arm motor and intake servo
            opMode.telemetry.addData("Test: ", "Intake arm motor");
            opMode.telemetry.update();
            // Complete the final position of the intake arm
            robot.intakeArmMotorL.setTargetPosition(300);
            robot.intakeArmMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeArmMotorR.setTargetPosition(300);
            robot.intakeArmMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeArmMotorL.setPower(0.25);
            robot.intakeArmMotorR.setPower(0.25);
            while (robot.intakeArmMotorR.isBusy() && robot.intakeArmMotorL.isBusy()) {
                // Do nothing
            }
            robot.intakeArmMotorL.setPower(0);
            robot.intakeArmMotorR.setPower(0);

            robot.intakeServo.setPower(1);
            opMode.sleep(1000);
            robot.intakeServo.setPower(0);
            opMode.telemetry.addData("Test: ", "Intake arm & servo complete");
            opMode.telemetry.update();
        }



        while (opMode.opModeIsActive() && arrayTests[2] && !opMode.gamepad1.a) {
        }

        if (arrayTests[2] && opMode.opModeIsActive()) {
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
            //gyroTurn(DT_TURN_SPEED_LOW, 0, 45, DT_TURN_THRESHOLD_LOW, false, false, 4);

            //strafe(0.4, 3, false, 6);
            //gyroTurn(DT_TURN_SPEED_LOW, 0, 0, DT_TURN_THRESHOLD_LOW, false, false, 2);

            //followHeadingPID(0, 0.6, 30, 60, true, 7 /*seconds*/);
            //strafe(0.4, 3, false, 6);
            //opMode.sleep(1000);
            //strafe(0.4, 3, true, 6);

            //followHeadingPIDBack(0, -0.3, 30, false, 6);

            // TEST PARKING
            // First raise the arm
            robot.intakeArmMotorR.setTargetPosition(400);
            robot.intakeArmMotorL.setTargetPosition(400);
            robot.intakeArmMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeArmMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeArmMotorR.setPower(0.2);
            robot.intakeArmMotorL.setPower(0.2);
            while (opMode.opModeIsActive() &&
                   robot.intakeArmMotorR.isBusy() && robot.intakeArmMotorL.isBusy()){
                // do nothing
            }

            double distToWall = robot.getDistanceCM(robot.distanceBack, 190, 0.5)/2.54; // in inches
            Log.v("BOK", "Dist to back wall (inches) " + distToWall);

            // Start raising the intake lift and lower the latching lift
            SetupArmThread setupArmThread = new SetupArmThread(opMode, false);
            setupArmThread.start();

            // Move to target distance of 60cm
            followHeadingPIDWithDistanceBack(0, -0.5, 60, 7/*sec*/);

            // Once distance reached, dump the marker
            dumpMarker();
            Log.v("BOK", "Dumping marker completed in " +
                    String.format("%.2f", BoKAuto.runTimeOpMode.seconds()));
            double distToMoveBack = Math.max(distToWall + 5, 56);
            double currentHeading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES).thirdAngle;
            // Turn away from our sampling sphere
            gyroTurn(DT_TURN_SPEED_LOW,
                    currentHeading,
                    0, DT_TURN_THRESHOLD_LOW, false, false, 3/*seconds*/);

            // Move forwards distToWall + 10 inches, detect bump with the crater wall
            followHeadingPID(0, 0.5 + 0.1,
                    0.8*distToMoveBack, distToMoveBack, true, 7 /*seconds*/);
            try {
                setupArmThread.join();
            } catch (InterruptedException e) {

            }

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

        while (opMode.opModeIsActive() && arrayTests[8] && !opMode.gamepad1.a) {
        }

        if (arrayTests[8] && opMode.opModeIsActive()) {

            // CameraDevice.getInstance().setFlashTorchMode(true);

            int numPics = 0;
            boolean x = false;
            boolean b = false;
            double currServoPos = robot.dumperRotateServo.getPosition();
            Log.v("BOK", "Init servo pos at " + String.format("%.2f" , currServoPos));
            while (opMode.opModeIsActive()) {
                if (opMode.gamepad1.x && !x) {
                    currServoPos = robot.dumperRotateServo.getPosition() + 0.1;
                    robot.dumperRotateServo.setPosition(currServoPos);
                    Log.v("BOK", "Up servo pos at " + String.format("%.2f" , currServoPos));
                }
                x = opMode.gamepad1.x;

                if (opMode.gamepad1.b && !b) {
                    currServoPos = robot.dumperRotateServo.getPosition() - 0.1;
                    robot.dumperRotateServo.setPosition(currServoPos);
                    Log.v("BOK", "Down servo pos at " + String.format("%.2f" , currServoPos));
                }
                b = opMode.gamepad1.b;
            }
        } // arrayTests[7]: take picture
    }
}
