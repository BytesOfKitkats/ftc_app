package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

/**
 * Created by krish on 6/14/2018.
 */
@TeleOp(name="Telop Arcade", group="Demo")
@Disabled
public class TeleopSmallBotArcade extends LinearOpMode {
    private DcMotor left, right;
    float lPwr, rPwr, xVal, yVal, turn;
    @Override
    public void runOpMode() throws InterruptedException
    {
        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive())
        {
            xVal = (float) (-gamepad1.left_stick_x*0.75);
            yVal = (float) (-gamepad1.left_stick_y*0.75);
            turn = (float) (-gamepad1.right_stick_x*0.75);
            lPwr = yVal + xVal;
            rPwr = yVal - xVal;

            if (Math.abs(turn) <= 0.25)
            {
                left.setPower(lPwr);
                right.setPower(rPwr);
            }

            else if (turn < -0.25)
                right.setPower(-turn);

            else if(turn > 0.25)
                left.setPower(turn);

            else
            {
                left.setPower(0);
                right.setPower(0);
            }

            telemetry.addData("left: ", lPwr + "right: " + rPwr);
            telemetry.update();
            sleep(10);
        }
    }
}
