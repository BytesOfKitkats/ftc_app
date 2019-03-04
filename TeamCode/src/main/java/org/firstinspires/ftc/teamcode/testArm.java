package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
@Autonomous(group="test", name="intakeArm Only")
//@Disabled
public class testArm extends LinearOpMode {
    DcMotor intakeArmMotorL;
    DcMotor intakeArmMotorR;
    DcMotor dumperSlideMotor;
    Servo dumperRotateServo;
    ElapsedTime runTime;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeArmMotorL = hardwareMap.dcMotor.get("inAL");
        intakeArmMotorR = hardwareMap.dcMotor.get("inAR");
        dumperSlideMotor = hardwareMap.dcMotor.get("duDC");
        dumperRotateServo = hardwareMap.servo.get("duS");
        runTime = new ElapsedTime();

        intakeArmMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intakeArmMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeArmMotorL.setPower(0);

        intakeArmMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeArmMotorR.setPower(0);

        dumperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dumperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dumperSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        dumperSlideMotor.setPower(0);

        double DUMPER_ROTATE_SERVO_INIT  = 0.35;
        double dPos, lastPos = 0;
        dumperRotateServo.setPosition(DUMPER_ROTATE_SERVO_INIT);

        waitForStart();

        //moveIntakeArmPID(840, -0.8, -1.26, 2.2);
        intakeArmMotorL.setTargetPosition(300);//375
        intakeArmMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotorR.setTargetPosition(300);
        intakeArmMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeArmMotorL.setPower(0.25);
        intakeArmMotorR.setPower(0.25);
        while (intakeArmMotorR.isBusy() && intakeArmMotorL.isBusy()) {
            // Do nothing
        }

        intakeArmMotorL.setPower(0);
        intakeArmMotorR.setPower(0);
        intakeArmMotorL.setTargetPosition(intakeArmMotorL.getCurrentPosition());
        intakeArmMotorR.setTargetPosition(intakeArmMotorR.getCurrentPosition());

        dumperSlideMotor.setTargetPosition(6250);
        dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dumperSlideMotor.setPower(0.8);

        while (dumperSlideMotor.isBusy()) {
           //intakeArmMotorL.setPower(0.4);
           //intakeArmMotorR.setPower(0.4);
        }
        Log.v("BOK", "Enc: "+ dumperSlideMotor.getCurrentPosition());

        intakeArmMotorL.setTargetPosition(0);
        intakeArmMotorR.setTargetPosition(0);
        intakeArmMotorL.setPower(0.05);
        intakeArmMotorR.setPower(0.05);
        while (intakeArmMotorR.isBusy() && intakeArmMotorL.isBusy()) {
            // Do nothing
        }
        intakeArmMotorL.setPower(0);
        intakeArmMotorR.setPower(0);

        Log.v("BOK", "IntakeR Enc: 0 " + intakeArmMotorR.getCurrentPosition());
        Log.v("BOK", "IntakeL Enc: 0 " + intakeArmMotorL.getCurrentPosition());

        // move the arm back out
        boolean a_dn_pressed = false;
        boolean a_stage_2 = false;
        boolean y_up_pressed = false;
        boolean y_stage_2 = false;
        boolean y_stage_3 = false;

        while (opModeIsActive()) {
            if (gamepad1.a && !a_dn_pressed && !a_stage_2) {
                a_dn_pressed = true;
                dumperRotateServo.setPosition(DUMPER_ROTATE_SERVO_INIT);
                intakeArmMotorL.setTargetPosition(400);
                intakeArmMotorR.setTargetPosition(400);
                intakeArmMotorL.setPower(0.12); // from slanting position to straight up
                intakeArmMotorR.setPower(0.12);
            }

            if (a_dn_pressed) {
                if (intakeArmMotorR.isBusy() && intakeArmMotorL.isBusy()) {
                    // do nothing
                }
                else {
                    Log.v("BOK", "IntakeR Enc Dn 400: " + intakeArmMotorR.getCurrentPosition());
                    Log.v("BOK", "IntakeL Enc Dn 400: " + intakeArmMotorL.getCurrentPosition());
                    intakeArmMotorL.setTargetPosition(900);
                    intakeArmMotorR.setTargetPosition(900);
                    intakeArmMotorL.setPower(0.1); // from slanting position to straight up
                    intakeArmMotorR.setPower(0.1);
                    a_stage_2 = true;
                    a_dn_pressed = false;
                }
            }
            if (a_stage_2) {
                if (intakeArmMotorR.isBusy() && intakeArmMotorL.isBusy()) {
                    // do nothing
                }
                else {
                    intakeArmMotorL.setPower(0);
                    intakeArmMotorR.setPower(0);
                    Log.v("BOK", "IntakeR Enc Dn 900: " + intakeArmMotorR.getCurrentPosition());
                    Log.v("BOK", "IntakeL Enc Dn 900: " + intakeArmMotorL.getCurrentPosition());
                    a_stage_2 = false; // the arm is on the floor due to gravity
                }
            }

            if (gamepad1.y && !y_up_pressed && !y_stage_2 && !y_stage_3) {
                y_up_pressed = true;
                Log.v("BOK", "IntakeR Enc Up Start: " + intakeArmMotorR.getCurrentPosition());
                Log.v("BOK", "IntakeL Enc Up Start: " + intakeArmMotorL.getCurrentPosition());
                lastPos = intakeArmMotorR.getCurrentPosition();
                intakeArmMotorL.setTargetPosition(600);
                intakeArmMotorR.setTargetPosition(600);
                intakeArmMotorL.setPower(0.3); // from slanting position to straight up
                intakeArmMotorR.setPower(0.3);
            }

            dPos = intakeArmMotorR.getCurrentPosition() - lastPos; // dPos is -ve
            lastPos = intakeArmMotorR.getCurrentPosition();
            if (y_up_pressed) {
                if (intakeArmMotorR.isBusy() && intakeArmMotorL.isBusy()) {
                    double newServoPos = dumperRotateServo.getPosition()
                            -0.000370*dPos; // note: dPos is -ves
                    //Log.v("BOK", "Servo Pos " + newServoPos + " dPos " + dPos);
                    dumperRotateServo.setPosition(newServoPos);
                    // do nothing
                } else {
                    Log.v("BOK", "IntakeR Enc Up 600: " + intakeArmMotorR.getCurrentPosition());
                    Log.v("BOK", "IntakeL Enc Up 600: " + intakeArmMotorL.getCurrentPosition());

                    intakeArmMotorL.setTargetPosition(400);
                    intakeArmMotorR.setTargetPosition(400);
                    intakeArmMotorL.setPower(0.2); // from slanting position to straight up
                    intakeArmMotorR.setPower(0.2);
                    y_stage_2 = true;
                    y_up_pressed = false;
                }
            }

            if (y_stage_2) {
                if (intakeArmMotorR.isBusy() && intakeArmMotorL.isBusy()) {
                    double newServoPos = dumperRotateServo.getPosition()
                            -0.000370*dPos; // note: dPos is -ve
                    //Log.v("BOK", "Servo Pos " + newServoPos + " dPos ");
                    dumperRotateServo.setPosition(newServoPos);
                } else {
                    Log.v("BOK", "IntakeR Enc up 400: " + intakeArmMotorR.getCurrentPosition());
                    Log.v("BOK", "IntakeL Enc up 400: " + intakeArmMotorL.getCurrentPosition());

                    intakeArmMotorL.setTargetPosition(200);
                    intakeArmMotorR.setTargetPosition(200); // from straight up to slightly slanted
                    intakeArmMotorL.setPower(0.12);
                    intakeArmMotorR.setPower(0.12);
                    y_stage_2 = false;
                }
            }
        }
    }

    /**
     * moveIntakeArmPID
     * @param endPos final position (encoder count)
     * @param power
     * @param vTarget in enc/mSec
     * @param waitForSec in sec before timing out
     */
    protected void moveIntakeArmPID (int endPos, double power, double vTarget, double waitForSec)
    {
        double vEnc, err, sumErr = 0, dErrDT, dT, pid, powerApp,
                Kp = 0.7, Ki = 0.525, Kd = 0.2, time, lastTime = 0, lastErr = 0;
        int inPos = intakeArmMotorL.getCurrentPosition(); // should be 0
        int lastPos = inPos;

        //Runtime
        runTime.reset();
        String logString = "pos,lPos,dTime,vEnc,err,sumErr,lastErr,dErrDT,pid,speed\n";
        intakeArmMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeArmMotorL.setPower(power);
        intakeArmMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeArmMotorR.setPower(power);

        while (opModeIsActive() && (Math.abs(inPos - endPos) > 20) &&
                (runTime.seconds() < waitForSec)) {

            inPos = intakeArmMotorL.getCurrentPosition();
            time = runTime.milliseconds();
            dT = time - lastTime;
            vEnc = (inPos - lastPos)/dT;
            err = vEnc - vTarget;
            sumErr = 0.67*sumErr + err*dT;
            dErrDT = (err - lastErr)/dT;
            pid = Kp*err + Ki*sumErr + Kd*dErrDT;
            powerApp = power-pid;
            if(power < 0){
                powerApp = Range.clip(powerApp, -1.0, 0.0);
            }
            else {
                powerApp = Range.clip(powerApp, 0.0, 1.0);
            }
            intakeArmMotorL.setPower(powerApp);
            intakeArmMotorR.setPower(powerApp);
            lastErr = err;
            lastTime = time;
            lastPos = inPos;
            logString += inPos+","+lastPos+","+dT+","+vEnc+","+err+","+sumErr+","+lastErr+","+dErrDT+","+pid+","+(power-pid)+"\n";
            Log.v("BOK", "Intake arm pos " + inPos);
        }

        File file = AppUtil.getInstance().getSettingsFile("BoKMotorData.csv");
        ReadWriteFile.writeFile(file,
               logString);
        intakeArmMotorL.setPower(0);
        intakeArmMotorR.setPower(0);
        if (runTime.seconds() > waitForSec) {
            Log.v("BOK", "moveIntakeArmPID timed out!");
        }
    }
}
