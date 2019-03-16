package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="BOK LIFT", group="Test")
public class latchTest extends LinearOpMode {
    DcMotor hangMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        hangMotor = hardwareMap.dcMotor.get("ha");
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotor.setPower(0);

        waitForStart();

        hangMotor.setPower(0.95);
        while (opModeIsActive() && (hangMotor.getCurrentPosition() < 1610)){
            Log.v("BOK", "Lift pos " + hangMotor.getCurrentPosition());
        }
        hangMotor.setPower(0);
    }
}
