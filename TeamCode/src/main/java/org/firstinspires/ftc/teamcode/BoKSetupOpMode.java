package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import android.util.Log;

/**
 * Created by Krishna Saxena on 11/1/2017.
 */
@TeleOp(name="BOK SETUP", group= "BoKZ")
//@Disabled
public class BoKSetupOpMode extends LinearOpMode
{
    BoKHardwareBot robot = new BoKMecanumDT();
    class InitRelicArmThread extends Thread {
        @Override
        public void run()
        {
            robot.initRelicArm();
        }
    }

    public void runOpMode()
    {
        boolean test_sensors = false, test_relic_arm = false;
        robot.initHardware(this);

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        waitForStart();
        
        while(opModeIsActive()) {
            // GAMEPAD 1 CONTROLS
            // X:                    Sensor values
            // Y:                    Test DC Motors
            if (gamepad1.x || test_sensors) {
                test_sensors = true;
                float[] hsvValues = robot.getHue(robot.colorNear);
                telemetry.addData("Near", "H: " + String.format("%.2f", hsvValues[0]) +
                        ", S: " + String.format("%.2f", hsvValues[1]) +
                        ", V: " + String.format("%.2f", hsvValues[2]) +
                        ", A: " + robot.colorNear.alpha() +
                        ", D: " + String.format("%.2f",
                        robot.distNear.getDistance(DistanceUnit.CM)));

                hsvValues = robot.getHue(robot.colorFar);
                telemetry.addData("Far", "H: " + String.format("%.2f", hsvValues[0]) +
                        ", S: " + String.format("%.2f", hsvValues[1]) +
                        ", V: " + String.format("%.2f", hsvValues[2]) +
                        ", A: " + robot.colorFar.alpha() +
                        ", D: " + String.format("%.2f",
                        robot.distFar.getDistance(DistanceUnit.CM)));

                hsvValues = robot.getHue(robot.colorBottom);
                telemetry.addData("Bottom", "H: " + String.format("%.2f", hsvValues[0]) +
                        ", S: " + String.format("%.2f", hsvValues[1]) +
                        ", V: " + String.format("%.2f", hsvValues[2]) +
                        ", A: " + robot.colorFar.alpha());

                telemetry.addData("Front", String.format("%.2f",
                        robot.getDistanceCM(robot.mb1240Front)));

                telemetry.addData("Back", String.format("%.2f",
                        robot.getDistanceCM(robot.mb1240Back)));

                telemetry.addData("SideR", String.format("%.2f",
                        robot.getDistanceCM(robot.mb1240SideR)));

                telemetry.addData("SideL", String.format("%.2f",
                        //robot.getDistanceCM(robot.mb1240SideL)));
                        robot.getDistanceCM(robot.mb1240SideL)));
            }

            if (gamepad1.y) {
                test_sensors = false;
                robot.leftRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftRoller.setTargetPosition(250);
                robot.leftRoller.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftRoller.setPower(0.2);

                while (opModeIsActive() && robot.leftRoller.isBusy()) {

                }
                robot.leftRoller.setPower(0);
                robot.leftRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.rightRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightRoller.setTargetPosition(250);
                robot.rightRoller.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightRoller.setPower(0.2);

                while (opModeIsActive() && robot.rightRoller.isBusy()) {

                }
                robot.rightRoller.setPower(0);
                robot.rightRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.flipperLift.setTargetPosition(250);
                robot.flipperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.flipperLift.setPower(0.2);
                while (opModeIsActive() && robot.flipperLift.isBusy()) {

                }
                robot.flipperLift.setPower(0);
                robot.flipperLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.a && !test_relic_arm) {
                InitRelicArmThread initRA = new InitRelicArmThread();
                initRA.start();
                test_relic_arm = true;
                try {
                    initRA.join();
                } catch (InterruptedException e) {

                }
            }
            if (gamepad1.b) {
                test_relic_arm = false;
            }

            if (gamepad2.y){
                robot.jewelFlicker.setPosition(robot.JF_FINAL);
                robot.jewelArm.setPosition(robot.JA_INIT);
            }

            if (gamepad2.a) {
                robot.jewelArm.setPosition(robot.JA_FINAL);
            }

            if (gamepad2.x) {
                //robot.jewelFlicker.setPosition(0.3);
                sleep(250);
                robot.jewelFlicker.setPosition(robot.JF_LEFT);
            }

            if (gamepad2.b) {
                //robot.jewelFlicker.setPosition(0.6);
                sleep(250);
                robot.jewelFlicker.setPosition(robot.JF_RIGHT);
            }

            telemetry.update();

            robot.waitForTick(BoKHardwareBot.WAIT_PERIOD);
        }
    }
}
