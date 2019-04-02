package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BoKAutoSetup extends BoKAutoCommon {

    // Constructor
    public BoKAutoSetup()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    boolean x = false, y = false;
    @Override
    public void runSoftware()
    {
        if (opMode.gamepad1.x && !x) {
            x = true;
            robot.intakeLeftServo.setPosition(robot.INTAKE_LEFT_SERVO_MID);
            robot.intakeRightServo.setPosition(robot.INTAKE_RIGHT_SERVO_MID);
            robot.intakeSlideMotor.setTargetPosition(-150);
            robot.intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeSlideMotor.setPower(0.7);
            while (robot.intakeSlideMotor.isBusy()) {
            }
            robot.intakeSlideMotor.setPower(0);
        }

        if (opMode.gamepad1.y && !y) {
            y = true;
            robot.hangMotor.setTargetPosition(-robot.HANG_LIFT_HIGH_POS + 50);
            robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangMotor.setPower(0.8);
            while (robot.hangMotor.isBusy()) {

            }
            robot.hangMotor.setPower(0);
        }
    }
}
