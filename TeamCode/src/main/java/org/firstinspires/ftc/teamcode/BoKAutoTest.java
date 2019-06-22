package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BoKAutoTest extends BoKAutoCommon {

    // Constructor
    public BoKAutoTest()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        // specify coefficients/gains
        PIDCoefficients coeffs = new PIDCoefficients(0.1, 0.0, 0.01);
        // create the controller
        PIDFController controller = new PIDFController(coeffs);

        // specify the setpoint
        controller.setTargetPosition(0);

        // in each iteration of the control loop
        // measure the position or output variable
        // apply the correction to the input variable
        //double correction = controller.update();
    }

    public void runSoftwareOld()
    {
        boolean[] arrayTests = {
                true, // IMU
                false, // DT motors
                false, // intake
                false, // dumper
                false, // latch
                false, // marker
                false, // distance sensor
                false, // take picture
                false  //auto test
        };


        if (arrayTests[0]) {
            Orientation angles;
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES);
            while (opMode.opModeIsActive() && !opMode.gamepad1.x) {
                opMode.telemetry.addData("Gyro ", angles.thirdAngle);
                opMode.telemetry.update();
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES);
            }
        } // if (arrayTests[0])

        while (opMode.opModeIsActive() && arrayTests[1] && !opMode.gamepad1.a) {
        }

        if (arrayTests[1] && opMode.opModeIsActive()) {
            // move (test the 4 DC motors
            opMode.telemetry.addData("Test: ", "Moving forward");
            opMode.telemetry.update();
            robot.testDTMotors();//move(0.5, 0.5, 12, true, 6);
            opMode.telemetry.addData("Test: ", "Moving forward complete");
            opMode.telemetry.update();
            opMode.sleep(2000);
        }

        while (opMode.opModeIsActive() && arrayTests[2] && !opMode.gamepad1.a) {
        }

        if (arrayTests[2] && opMode.opModeIsActive()) {
            // Test intake arm motor and intake motor
            opMode.telemetry.addData("Test: ", "Intake arm motor");
            opMode.telemetry.update();
            robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_UP);
            robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_UP);
            robot.intakeSlideMotor.setTargetPosition(-200);
            robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeSlideMotor.setPower(0.7);
            while (robot.intakeSlideMotor.isBusy()) {

            }
            robot.intakeSlideMotor.setPower(0);
            robot.intakeMotor.setPower(0.7);
            opMode.sleep(2000);
            robot.intakeMotor.setPower(0);
            robot.intakeGateServo.setPosition(robot.INTAKE_GATE_SERVO_OPEN);
            opMode.sleep(500);
            robot.intakeGateServo.setPosition(robot.INTAKE_GATE_SERVO_CLOSED);
            opMode.telemetry.addData("Test: ", "Intake arm complete");
            opMode.telemetry.update();
        } // if (arrayTests[1])

        while (opMode.opModeIsActive() && arrayTests[3] && !opMode.gamepad1.a) {
        }

        if (arrayTests[3] && opMode.opModeIsActive()) {
            // test Dumper lift motor
            opMode.telemetry.addData("Test: ", "Moving dumper lift");
            opMode.telemetry.update();
            robot.dumperSlideMotor.setTargetPosition(1120);
            robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.dumperSlideMotor.setPower(0.95);
            while (opMode.opModeIsActive() && robot.dumperSlideMotor.isBusy()) {
                Log.v("BOK", "Dumper slide: " + robot.dumperSlideMotor.getCurrentPosition());
            }

            Log.v("BOK", "Dumper. final " + robot.dumperSlideMotor.getCurrentPosition());
            robot.dumperSlideMotor.setPower(0.1);

            boolean dump = false;
            while (opMode.opModeIsActive() && !opMode.gamepad1.x) {
                //Log.v("BOK", "Dumper slide: " + robot.dumperSlideMotor.getCurrentPosition());
                if (opMode.gamepad1.y && !dump) {
                    dump = opMode.gamepad1.y;
                    robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_FINAL);
                }
                if (opMode.gamepad1.b && dump) {
                    dump = false;
                    robot.dumperGateServo.setPosition(robot.DUMPER_GATE_SERVO_INIT);
                }
            }

            robot.dumperSlideMotor.setTargetPosition(0);
            robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.dumperSlideMotor.setPower(0.9);

            Log.v("BOK", "Dumper slide done: " + robot.dumperSlideMotor.getCurrentPosition());
            opMode.telemetry.addData("Test: ", "Moving dumper lift complete");
            opMode.telemetry.update();
        }

        while (opMode.opModeIsActive() && arrayTests[4] && !opMode.gamepad1.a) {
        }

        if (arrayTests[4] && opMode.opModeIsActive()) {
            // Test hanging lift and hang hook servo
            opMode.telemetry.addData("Test: ", "Hanging lift motor");
            opMode.telemetry.update();

            robot.hangMotor.setTargetPosition(robot.HANG_LIFT_HIGH_POS);
            robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangMotor.setPower(0.5);
            while (opMode.opModeIsActive() && robot.hangMotor.isBusy()) {

            }
            robot.hangMotor.setPower(0);
            opMode.telemetry.addData("Test: ", "Moving hanging complete");
            opMode.telemetry.update();
            opMode.sleep(1000);
        }

        while (opMode.opModeIsActive() && arrayTests[5] && !opMode.gamepad1.a) {
        }

        if (arrayTests[5] && opMode.opModeIsActive()) {
            // Test marker servo
            opMode.telemetry.addData("Test: ", "Marker servo");
            opMode.telemetry.update();
            robot.markerServo.setPosition(robot.MARKER_SERVO_FINAL);
            opMode.sleep(1000);
            robot.markerServo.setPosition(robot.MARKER_SERVO_INIT);
        }


        while (opMode.opModeIsActive() && arrayTests[6] && !opMode.gamepad1.a) {
        }

        if (arrayTests[6] && opMode.opModeIsActive()) {
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
            while (opMode.opModeIsActive() && !opMode.gamepad1.b) {
                double dist = robot.getDistanceCM(robot.distanceBack, 150, 2);
                opMode.telemetry.addData("Distance: ", dist);
                opMode.telemetry.update();
            }

            opMode.telemetry.addData("Test: ", "All tests complete");
            opMode.telemetry.update();
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

                }
                x = opMode.gamepad1.x;
            }
        } // arrayTests[7]

        while (opMode.opModeIsActive() && arrayTests[8] && !opMode.gamepad1.a) {

        }

        if (arrayTests[8] && opMode.opModeIsActive()) {
            // Autonomous tests

            while (opMode.opModeIsActive() &&
                    (robot.hangMotor.getCurrentPosition() < robot.HANG_LIFT_HIGH_POS) &&
                    (runTime.seconds() < 6)) {
                // Do nothing while the robot is moving down
                // Log.v("BOK", "hang enc: " + robot.hangMotor.getCurrentPosition());
            }
            robot.hangMotor.setPower(0);
            if (runTime.seconds() >= 6) {
                Log.v("BOK", "hang lift timed out");
            }

        } // arrayTests[7]

    }
}
