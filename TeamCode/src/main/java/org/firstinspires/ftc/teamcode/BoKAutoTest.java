package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.vuforia.CameraDevice;

public class BoKAutoTest extends BoKAutoCommon {

    // Constructor
    public BoKAutoTest()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {


        /*
        opMode.telemetry.addData("Test: ", "Moving forward");
        opMode.telemetry.update();
        move(0.5, 0.5, 12, true, 6);
        opMode.telemetry.addData("Test: ", "Moving forward complete");
        opMode.telemetry.update();
        opMode.sleep(2000);*/
        /*
        opMode.telemetry.addData("Test: ", "Moving dumper lift");
        opMode.telemetry.update();
        robot.dumperSlideMotor.setTargetPosition(robot.DUMPER_SLIDE_FINAL_POS);
        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dumperSlideMotor.setPower(0.9);

        boolean dump = false;
        double nextPos = robot.DUMPER_ROTATE_SERVO_INIT - 0.05;
        while(opMode.opModeIsActive() && !opMode.gamepad1.x){
            Log.v("BOK", "Dumper slide: " + robot.dumperSlideMotor.getCurrentPosition());
            if (opMode.gamepad1.a && !dump) {
                dump = opMode.gamepad1.a;
                //robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_FINAL);
            }
            if (dump && (nextPos > robot.DUMPER_ROTATE_SERVO_FINAL)) {
                robot.dumperRotateServo.setPosition(nextPos);
                nextPos -= 0.005;
            }
            if (opMode.gamepad1.b && dump) {
                dump = false;
                nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
                robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
            }
        }

        robot.dumperSlideMotor.setTargetPosition(0);
        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dumperSlideMotor.setPower(0.9);

        Log.v("BOK", "Dumper slide done: " + robot.dumperSlideMotor.getCurrentPosition());

        opMode.telemetry.addData("Test: ", "Moving dumper lift complete");
        opMode.telemetry.update();
        opMode.sleep(2000);
        */
       /* robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_INIT);
        gyroTurn(DT_TURN_SPEED_LOW, 0, 45, DT_TURN_THRESHOLD_LOW,
                false, false, 4);
                */
       //dumpMarker();
        //strafe(0.5, 3, false, 6);
        //dropIntakeArmAndExtend();
        //robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_FINAL);
        //gyroTurn(0.1, 0, -5, DT_TURN_THRESHOLD_LOW, false, false, 5);
        /*followHeadingPIDBack(0, -0.3, 30,
                false, 6);
        gyroTurn(0.1, -5, 0, DT_TURN_THRESHOLD_LOW, false, false, 3);
        followHeadingPID(0, 0.3, 30,
                false, 8);*/
        //strafe(0.5, 3, false, 15);
        //robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_FINAL);
        //opMode.sleep(500);
        //gyroTurn(0.6, 0, 45, DT_TURN_THRESHOLD_LOW+1, false, false, 6);
        followHeadingPID(0, 0.5, 45, true, 20);
        //followHeadingPIDWithDistanceBack(45, -0.5, 60, false, 20);
        /*
        opMode.telemetry.addData("Test: ", "Moving intake rotate");
        opMode.telemetry.update();
        robot.intakeArmMotor.setTargetPosition(-1000);//600 to go down extended//1200to go down init
        robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeArmMotor.setPower(0.3);
        while(opMode.opModeIsActive()){
            if(robot.intakeArmMotor.getCurrentPosition() > -980){
                opMode.telemetry.addData("Test: ", "Moving intake rotate past 590");
                opMode.telemetry.update();
            }
        }
        */

        /*
        DcMotorEx motorExLeft = (DcMotorEx)opMode.hardwareMap.get(DcMotor.class, "inA");
        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDFCoefficients pidOrig = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(2.5, 0.1, 0.2, 0);
        motorExLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfNew);
        // display info to user.
        while(opMode.opModeIsActive()) {
            //opMode.telemetry.addData("Runtime", "%.03f", opMode.getRuntime());
            if(opMode.gamepad1.x)
                pidOrig = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if(opMode.gamepad1.y)
                pidOrig = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            opMode.telemetry.addData("P,I,D, F (orig)", "%.04f, %.04f, %.0f, %.04f",
                    pidOrig.p, pidOrig.i, pidOrig.d, pidOrig.f);
            opMode.telemetry.update();
        }
        */

        //robot.moveIntakeArmPID(400, 0.25, 0.2, 10);

        /*
        opMode.telemetry.addData("Test: ", "Moving intake sweeper");
        opMode.telemetry.update();
        runTime.reset();
        robot.intakeMotor.setPower(0.5);
        while(opMode.opModeIsActive() && (runTime.seconds()<2)){

        }
        robot.intakeMotor.setPower(0);
        opMode.telemetry.addData("Test: ", "Moving intake sweeper complete");
        opMode.telemetry.update();
        opMode.sleep(2000);
        */

        /*
        opMode.telemetry.addData("Test: ", "Moving hanging");
        opMode.telemetry.update();
        robot.hangMotor.setTargetPosition(robot.HANG_LIFT_HIGH_POS);
        robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hangMotor.setPower(0.5);
        while(opMode.opModeIsActive() && robot.hangMotor.isBusy()){

        }
        robot.hangMotor.setPower(0);
        opMode.telemetry.addData("Test: ", "Moving hanging complete");
        opMode.telemetry.update();
        opMode.sleep(2000);
        */

        /*
        CameraDevice.getInstance().setFlashTorchMode(true);

        int numPics = 0;
        boolean x = false;
        boolean y = false;
        boolean a = false;
        while (opMode.opModeIsActive()){
            if (opMode.gamepad1.x && !x){
                takePicture("SamplingImage_"+numPics+".png");
                numPics++;

            }
            x  = opMode.gamepad1.x;

            if (opMode.gamepad1.y && !y){
                robot.hangMotor.setTargetPosition(robot.HANG_LIFT_HIGH_POS);
                robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.hangMotor.setPower(0.9);
            }

            if (opMode.gamepad1.a && !a){
                CameraDevice.getInstance().setFlashTorchMode(false);
            }
            if(!robot.hangMotor.isBusy())
                robot.hangMotor.setPower(0);

            opMode.telemetry.addData("distance sensor ", robot.getDistanceCM(robot.distanceBack, 200, 0.5) + "in cm");
            opMode.telemetry.update();
        }
        */


/*
        runTime.reset();

        robot.intakeArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intakeArmMotor.setTargetPosition(100);
        robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Log.v("BOK", "encoder count " + robot.intakeArmMotor.getCurrentPosition() + ", " + runTime.seconds());
        for(int i = 2; i <= 3; i++){
            //opMode.sleep(100);


            robot.intakeArmMotor.setPower(0.8);
            while (robot.intakeArmMotor.isBusy()){

            }
            robot.intakeArmMotor.setPower(0);
            Log.v("BOK", "encoder count loop 1 " + robot.intakeArmMotor.getCurrentPosition() + ", " + runTime.seconds());
            robot.intakeArmMotor.setTargetPosition(100*i);
        }
        Log.v("BOK", "encoder count 2 " + robot.intakeArmMotor.getCurrentPosition());
        opMode.sleep(100);
        for(int i = 4; i <= 8; i++){
            robot.intakeArmMotor.setTargetPosition(100*i);
            robot.intakeArmMotor.setPower(0.1);
            while (robot.intakeArmMotor.isBusy()){

            }
            robot.intakeArmMotor.setPower(0);
            Log.v("BOK", "encoder count loop 2 " + robot.intakeArmMotor.getCurrentPosition() + ", " + runTime.seconds());
        }

        opMode.sleep(5000);
        robot.dumperRotateServo.setPosition(robot.DUMPER_RECEIVE_SERVO);

        Log.v("BOK", "encoder count 3 " + robot.intakeArmMotor.getCurrentPosition() + ", " + runTime.seconds());
        for(int i = 7; i >= 4; i--){
            //opMode.sleep(100);
            robot.intakeArmMotor.setTargetPosition(100*i);
            robot.intakeArmMotor.setPower(-0.9);
            while (robot.intakeArmMotor.isBusy()){

            }
            robot.intakeArmMotor.setPower(0);
            Log.v("BOK", "encoder count loop 3 " + robot.intakeArmMotor.getCurrentPosition() + ", " + runTime.seconds());
        }

        Log.v("BOK", "encoder count 4 " + robot.intakeArmMotor.getCurrentPosition() + ", " + runTime.seconds());
        opMode.sleep(100);
        for(int i = 3; i >= 1; i--) {
            //opMode.sleep(100);
            robot.intakeArmMotor.setTargetPosition(100 * i);
            robot.intakeArmMotor.setPower(-0.2);
            while (robot.intakeArmMotor.isBusy()){

            }
            robot.intakeArmMotor.setPower(0);
            Log.v("BOK", "encoder count loop 4 " + robot.intakeArmMotor.getCurrentPosition() + ", " + runTime.seconds());
        }

        robot.intakeArmMotor.setTargetPosition(0);
        robot.intakeArmMotor.setPower(-0.2);

        while (opMode.opModeIsActive()){
            Log.v("BOK", "encoder count final " + robot.intakeArmMotor.getCurrentPosition() + ", " + runTime.seconds());
        }
        */
        //setPowerToDTMotorsStrafeForTime(0.5, 2, false);
        //robot.distanceBack.resetDeviceConfigurationForOpMode();

        /*
        int pos = 0;
        robot.dumperRotateServo.setPosition(robot.DUMPER_RECEIVE_SERVO);

        boolean dump = false;
        while (opMode.opModeIsActive()){
            if(opMode.gamepad2.y){
                robot.moveIntakeArmPID(1000, 0.5, 0.5, 4);
                pos = 1000;
                pos = 1000;
            }
            else if (opMode.gamepad2.a){
                robot.moveIntakeArmPID(0, -0.5, -0.7, 4);
                pos = 0;
            }
            else if (!robot.isRunningIntakeArmPID) {
                robot.intakeArmMotor.setTargetPosition(pos);
                robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.intakeArmMotor.setPower(0.5);
            }

            if (opMode.gamepad2.left_bumper && !dump){
                DumpSlowly ds = new DumpSlowly();
                ds.run();
            }
        }
        */

        //robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_FINAL);
        //followHeadingPIDWithDistance(0, -0.5, 40, false, false, 6);
/*
        boolean x = false;
        boolean y = false;
        while (opMode.opModeIsActive()){
            if(opMode.gamepad1.x && !x){
                //robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_FINAL);
                robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_FINAL);
                x = true;
            }
            if(opMode.gamepad1.y && !y){
                //robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_INIT);
                robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_INIT);
                y = true;
            }
            Log.v("BOK", "Distance to wall " + robot.getDistanceCM(robot.distanceBack, 200, 0.5));
        }*/
    }

    class DumpSlowly implements Runnable
    {
        DumpSlowly()
        {

        }
        public void run()
        {
            double currDuRPos = 0.6;
            while (currDuRPos > robot.DUMPER_ROTATE_SERVO_FINAL){
                currDuRPos -= 0.05;
                robot.dumperRotateServo.setPosition(currDuRPos);
            }
        }
    }
}
