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

    public void runOpMode()
    {
        robot.initHardware(this);
        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        waitForStart();
        
        while(opModeIsActive()) {
            // GAMEPAD 1 CONTROLS
            // X:                    Sensor values
            // Y:                    Test DC Motors
            if (gamepad1.x) {
                float[] hsvValues = robot.getHue(robot.colorNear);
                telemetry.addData("Near", "H: " + String.format("%.2f", hsvValues[0]) +
                        ", S: " + String.format("%.2f", hsvValues[1]) +
                        ", V: " + String.format("%.2f", hsvValues[2]) +
                        ", A: " + String.format("%.2f", robot.colorNear.alpha()) +
                        ", D: " + String.format("%.2f",
                        robot.distNear.getDistance(DistanceUnit.CM)));

                hsvValues = robot.getHue(robot.colorFar);
                telemetry.addData("Far", "H: " + String.format("%.2f", hsvValues[0]) +
                        ", S: " + String.format("%.2f", hsvValues[1]) +
                        ", V: " + String.format("%.2f", hsvValues[2]) +
                        ", A: " + String.format("%.2f", robot.colorFar.alpha()) +
                        ", D: " + String.format("%.2f",
                        robot.distNear.getDistance(DistanceUnit.CM)));

                hsvValues = robot.getHue(robot.colorBottom);
                telemetry.addData("Bottom", "H: " + String.format("%.2f", hsvValues[0]) +
                        ", S: " + String.format("%.2f", hsvValues[1]) +
                        ", V: " + String.format("%.2f", hsvValues[2]) +
                        ", A: " + String.format("%.2f", robot.colorFar.alpha()));

                telemetry.addData("Front", String.format("%.2f",
                        robot.getDistanceCM(robot.mb1240Front)));

                telemetry.addData("Back", String.format("%.2f",
                        robot.getDistanceCM(robot.mb1240Back)));

                telemetry.addData("Side", String.format("%.2f",
                        robot.getDistanceCM(robot.mb1240Side)));
            }

            if (gamepad1.y) {
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

                robot.relicSpool.setTargetPosition(250);
                robot.relicSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.relicSpool.setPower(0.2);
                while (opModeIsActive() && robot.relicSpool.isBusy()) {

                }
                robot.relicSpool.setPower(0);
                robot.relicSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            telemetry.update();

            robot.waitForTick(BoKHardwareBot.WAIT_PERIOD);
        }
    }
}
