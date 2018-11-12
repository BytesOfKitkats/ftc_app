package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedFar extends BoKAutoCommon {
    BoKAutoCubeLocation loc = BoKAutoCubeLocation.BOK_CUBE_UNKNOWN;
    double initDist = 10;
    private OpenGLMatrix location = null;

    // Constructor
    public BoKAutoRedFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
        far = true;
    }

    @Override
    public void runSoftware()
    {
        // land and identify gold
        Log.v("BOK", "Angle at start " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle);
        robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_HANG_TILT);
        loc = findCube();
        robot.hangMotor.setTargetPosition(2325);//8650
        robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangMotor.setPower(0.75);
        runTime.reset();
        while (robot.hangMotor.isBusy()&&(runTime.seconds()<10)){
            //Log.v("BOK", "hang enc: " + robot.hangMotor.getCurrentPosition());
        }
        robot.hangMotor.setPower(0);
        if (runTime.seconds() >= 10)
            Log.v("BOK", "hang lift timed out");

        robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_FINAL);
        opMode.sleep(250);
        Log.v("BOK", "Angle at end " + robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES).thirdAngle);
        move(0.5, 0.5, 2, true, 2);
        gyroTurn(0.5, 0, 0, DT_TURN_THRESHOLD_LOW,
                false, false, 5);

        // knock off gold
        moveRamp(0.5, initDist, true, 5);
        moveLiftDown liftDown = new moveLiftDown();
        liftDown.start();
        if (loc == BoKAutoCubeLocation.BOK_CUBE_LEFT){
            robot.samplerLeftServo.setPosition(robot.SAMPLER_LEFT_SERVO_FINAL);
            moveRamp(0.5, 10, true, 5);
            robot.samplerLeftServo.setPosition(robot.SAMPLER_LEFT_SERVO_INIT);
            move(0.5, 0.5,10, false, 5);
        }

        else if (loc == BoKAutoCubeLocation.BOK_CUBE_CENTER){
            //TODO how much intake needs to go in pulses
            moveIntake(0.3, 780);
            moveRamp(0.5, 5.5, true, 5);
            moveIntake(0.3, 10);
            moveRamp(0.5, 5.5, false, 5);
        }

        else {
            robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_FINAL);
            moveRamp(0.5, 16, true, 5);
            robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_INIT);
            moveRamp(0.5, 16, false, 5);
        }
        gyroTurn(0.5, 0, 75, DT_TURN_THRESHOLD_LOW,
               false, false, 5);
        opMode.sleep(1000);
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            Log.v("BOK", "Searching target " + trackable.getName());
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                Log.v("BOK", "Visible Target" + trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    location = robotLocationTransform;
                }
                break;
            }
        }
        double dist;
        if(targetVisible){
            VectorF translation = location.getTranslation();
            double y = translation.get(1)/mmPerInch + 60; // we want to be at (0, -60)
            double x = translation.get(0)/mmPerInch + 6;
            Log.v("BOK", String.format("{X, Y} = %.1f, %.1f",
                    x, y));
            dist = Math.hypot(x, y);
        }
        else {
            dist = 35;
            Log.v("BOK", "Did not lock into Vuforia dist is 31");
        }
        followHeadingPID(75, 0.5, dist, 6);
        gyroTurn(0.5, 75, 130, DT_TURN_THRESHOLD_LOW,
                false, false, 4);
        Log.v("BOK" , "Dist to front wall " +
                robot.getDistanceCM(robot.distanceFront, 40, 0.5));
        moveWithRangeSensor(0.5, 80, 160, true, 7);
        moveIntake(0.5, 780);
        sweepRoller(-1);
        opMode.sleep(1000);
        sweepRoller(0);
        moveIntake(0.5, 10);
        moveWithRangeSensor(0.5, 225, 250, true, 7);
    }
}
