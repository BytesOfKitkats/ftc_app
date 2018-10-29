package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    BoKAutoCubeLocation loc;
    double initDist = 18;

    // Constructor
    public BoKAutoRedFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
        far = true;
    }

    @Override
    public void runSoftware()
    {
        robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT+0.1);
        loc = findCube();
        robot.hangMotor.setTargetPosition(8900);
        robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangMotor.setPower(0.75);
        while (robot.hangMotor.isBusy()){

        }
        robot.hangMotor.setPower(0);

        robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_FINAL);

        moveRamp(0.5, 18, true, 5);
        moveLiftDown liftDown = new moveLiftDown();
        liftDown.start();

        if (loc == BoKAutoCubeLocation.BOK_CUBE_LEFT){
            robot.samplerLeftServo.setPosition(robot.SAMPLER_LEFT_SERVO_FINAL);
            moveRamp(0.5, 22, true, 5);
        }

        else if (loc == BoKAutoCubeLocation.BOK_CUBE_CENTER){
            moveIntake(0.3, 460);
            sweepRoller(1);
            moveRamp(0.5, 15, true, 5);
        }

        else {
            robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_FINAL);
            moveRamp(0.5, 22, true, 5);
        }

        sweepRoller(0);
        moveIntake(-0.6, 0);
        robot.samplerLeftServo.setPosition(robot.SAMPLER_LEFT_SERVO_INIT);
        robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_INIT);

        moveRamp(0.5, 7, true, 5);
    }
}
