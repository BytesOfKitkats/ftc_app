package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.graphics.Color;
import android.util.Log;

/**
 * Created by Krishna Saxena on 9/24/2016.
 */
public abstract class BoKHardwareBot
{
    // CONSTANTS
    protected static final int OPMODE_SLEEP_INTERVAL_MS_SHORT  = 10;
    protected static final double SPEED_COEFF_SLOW = 0.35;
    protected static final double SPEED_COEFF_FAST = 0.8;
    protected static final double SPEED_COEFF_TURN = 0.7;
    protected static final double GAME_STICK_DEAD_ZONE = 0.1;

    // DC Motors
    private static final String INTAKE_ARM_MOTOR_NAME   = "inA";
    private static final String INTAKE_MOTOR_NAME       = "in";
    private static final String DUMPER_SLIDE_MOTOR_NAME = "duS";
    private static final String HANG_MOTOR_NAME         = "ha";

    // Servos
   private static final String DUMPER_ROTATE_SERVO_NAME = "duR";
   private static final String HANG_HOOK_SERVO_NAME     = "haH";
   private static final String INTAKE_GATE_SERVO_NAME   = "inG";
   private static final String SAMPLER_SERVO_NAME       = "sa";
   private static final String MARKER_SERVO_NAME        = "ma";
   private static final String DISTANCE_ROTATE_SERVO_NAME = "dsR";

    // Servo positions
    protected static final double DUMPER_ROTATE_SERVO_INIT  = 0.6;
    protected static final double DUMPER_RECEIVE_SERVO      = 0.52;
    protected static final double DUMPER_ROTATE_SERVO_FINAL = 0.0;
    protected static final double HANG_HOOK_SERVO_INIT      = 0.7;
    protected static final double HANG_HOOK_SERVO_FINAL     = 0.07;
    protected static final double INTAKE_GATE_SERVO_INIT    = 0.5;
    protected static final double INTAKE_GATE_SERVO_FINAL   = 0;
    protected static final double SAMPLER_SERVO_INIT        = 0.5;
    protected static final double SAMPLER_SERVO_FINAL       = 0.95;
    protected static final double MARKER_SERVO_INIT         = 0.38;
    protected static final double MARKER_SERVO_FINAL        = 0.85;
    protected static final double DISTANCE_ROTATE_SERVO_INIT  = 0.5;
    protected static final double DISTANCE_ROTATE_SERVO_FINAL = 0.68;

    // Encoder positions
    protected static final int DUMPER_SLIDE_FINAL_POS    = 970;
    protected static final int INTAKE_ARM_DOWN_POS       = 400;
    protected static final int HANG_LIFT_HIGH_POS        = 2100;//2325;

    // Sensors
    private static final String IMU_TOP = "imu";        // IMU
    private static final String DISTANCE_SENSOR_BACK = "dsB";

    protected static final int WAIT_PERIOD = 40; // 40 ms

    LinearOpMode opMode; // current opMode

    // DC motors
    protected DcMotor intakeArmMotor;
    protected DcMotor intakeMotor;
    protected DcMotor dumperSlideMotor;
    protected DcMotor hangMotor;

    // Servos
    protected Servo   dumperRotateServo;
    protected Servo   hangHookServo;
    protected Servo   markerServo;
    protected Servo   intakeGateServo;
    protected Servo   samplerServo;
    protected Servo   distanceRotateServo;

    // Sensors
    protected BNO055IMU imu;
    protected AnalogInput distanceBack;

    private Orientation angles;

    // waitForTicks
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime runTime  = new ElapsedTime();

    public boolean isRunningIntakeArmPID = false;

    // return status
    protected enum BoKHardwareStatus
    {
        BOK_HARDWARE_FAILURE,
        BOK_HARDWARE_SUCCESS
    }

    /*
     * The initHardware() method initializes the hardware on the robot including the drive train.
     * It calls the abstract initDriveTrainMotors() and initMotorsAndSensors() methods.
     * Returns BOK_SUCCESS if the initialization is successful, BOK_FAILURE otherwise.
     */
    protected BoKHardwareStatus initHardware(LinearOpMode opMode)
    {
        this.opMode = opMode;
        // First initialize the drive train
        BoKHardwareStatus rc = initDriveTrainMotors();
        if (rc == BoKHardwareStatus.BOK_HARDWARE_SUCCESS) {
            // Next initialize the other motors and sensors
            rc = initMotorsAndSensors();
        }
        return rc;
    }

    /*
     * The initMotorsAndSensors() method initializes the motors (other than the drive train) and
     * sensors on the robot.
     */
    private BoKHardwareStatus initMotorsAndSensors()
    {
        // DC Motors
        intakeArmMotor = opMode.hardwareMap.dcMotor.get(INTAKE_ARM_MOTOR_NAME);
        if (intakeArmMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        intakeMotor = opMode.hardwareMap.dcMotor.get(INTAKE_MOTOR_NAME);
        if (intakeMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        dumperSlideMotor = opMode.hardwareMap.dcMotor.get(DUMPER_SLIDE_MOTOR_NAME);
        if (dumperSlideMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        hangMotor = opMode.hardwareMap.dcMotor.get(HANG_MOTOR_NAME);
        if (hangMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        // Servos
        samplerServo = opMode.hardwareMap.servo.get(SAMPLER_SERVO_NAME);
        if (samplerServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        dumperRotateServo = opMode.hardwareMap.servo.get(DUMPER_ROTATE_SERVO_NAME);
        if (dumperRotateServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        hangHookServo = opMode.hardwareMap.servo.get(HANG_HOOK_SERVO_NAME);
        if (hangHookServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        markerServo = opMode.hardwareMap.servo.get(MARKER_SERVO_NAME);
        if (markerServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        intakeGateServo = opMode.hardwareMap.servo.get(INTAKE_GATE_SERVO_NAME);
        if (intakeGateServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        distanceRotateServo = opMode.hardwareMap.servo.get(DISTANCE_ROTATE_SERVO_NAME);
        if (distanceRotateServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        // Sensors
        imu = opMode.hardwareMap.get(BNO055IMU.class, IMU_TOP);
        if(imu == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        distanceBack = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_BACK);
        if(distanceBack == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        // DC Motor initialization
        intakeArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeArmMotor.setPower(0);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setPower(0);

        //dumperSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        dumperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dumperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dumperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotor.setPower(0);

        // Servos initialization
        if (!opMode.getClass().getName().contains("Tele")) {
            samplerServo.setPosition(SAMPLER_SERVO_INIT);
            hangHookServo.setPosition(HANG_HOOK_SERVO_INIT);
            markerServo.setPosition(MARKER_SERVO_INIT);
            intakeGateServo.setPosition(INTAKE_GATE_SERVO_INIT);
            dumperRotateServo.setPosition(DUMPER_ROTATE_SERVO_INIT);
            distanceRotateServo.setPosition(DISTANCE_ROTATE_SERVO_INIT);
        }
        else {
            // Do nothing for Teleop so that the robot hardware does not move during
            // initialization
        }

        return BoKHardwareStatus.BOK_HARDWARE_SUCCESS;
    }

    protected void initializeImu()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //angles = new Orientation();
        imu.initialize(parameters);
    }

    // Initialization of drive train is protected but abstract
    protected abstract BoKHardwareStatus initDriveTrainMotors();

    // Using the drive train is public
    protected abstract void resetDTEncoders();
    protected abstract boolean areDTMotorsBusy();

    protected abstract void setPowerToDTMotors(double power);
    protected abstract void setPowerToDTMotors(double power, boolean forward);
    protected abstract void setPowerToDTMotors(double leftPower, double rightPower);
    //protected abstract void setPowerToDTMotorsStrafe(double power, boolean right);
    protected abstract void setModeForDTMotors(DcMotor.RunMode runMode);
    protected abstract void setOnHeading(double leftPower, double rightPower);

    // Autonomous driving
    protected abstract int startMove(double leftPower,
                                     double rightPower,
                                     double inches,
                                     boolean backward);

    protected abstract void startEncMove(double leftPower,
                                         double rightPower,
                                         int encCount,
                                         boolean forward);

    /*
    protected abstract int startStrafe(double power, double rotations,
                                       boolean right) throws UnsupportedOperationException;

    protected abstract int startStrafeWEnc(double power, double rotations,
                                       boolean right) throws UnsupportedOperationException;*/

    protected abstract void stopMove();

    protected abstract double getTargetEncCount(double targetDistanceInches);
    protected abstract double getAvgEncCount();
    protected abstract int getLFEncCount();
    protected abstract int getRFEncCount();

    protected abstract void testDTMotors();
    // Teleop driving
    protected abstract void moveRobot(double speedCoef);

    /*
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    protected void waitForTick(long periodMs)
    {
        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            }
            catch (InterruptedException e) {
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    protected double getAngle (){

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    protected double getDistanceCM (AnalogInput mb1240, double target, double time)
    {
        runTime.reset();
        double dist = mb1240.getVoltage() / 0.00189;
        while ((dist > target)&&(runTime.seconds() <= time))
            dist = mb1240.getVoltage() / 0.00189;
        return (runTime.seconds() > time) ? target : dist;
        //return mb1240.getVoltage() / 0.00189;
    }

    /**
     *
     * @param endPos final position
     * @param power
     * @param vTarget in enc/mSec
     * @param waitForSec in sec
     */
    protected void moveIntakeArmPID (int endPos, double power, double vTarget, double waitForSec){
        double vEnc, err, sumErr = 0, dErrDT, dT, pid,
                Kp = 0.7, Ki = 0.525, Kd = 0.2, time, lastTime = 0, lastErr = 0, powerApp;
        int inPos = intakeArmMotor.getCurrentPosition(),
                lastPos = intakeArmMotor.getCurrentPosition();
        //Runtime
        runTime.reset();

        //String logString = "pos,lPos,dTime,vEnc,err,sumErr,lastErr,dErrDT,pid,speed\n";
        opMode.telemetry.addData("BOK ", "Starting PID");
        opMode.telemetry.update();
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeArmMotor.setPower(power);

        while (opMode.opModeIsActive() && (Math.abs(inPos - endPos) > 10) &&
                (runTime.seconds() < waitForSec)) {

            inPos = intakeArmMotor.getCurrentPosition();
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
            else
                powerApp = Range.clip(powerApp, 0.0, 1.0);
            intakeArmMotor.setPower(powerApp);
            lastErr = err;
            lastTime = time;
            lastPos = inPos;
            //logString += inPos+","+lastPos+","+dT+","+vEnc+","+err+","+sumErr+","+lastErr+","+dErrDT+","+pid+","+(power-pid)+"\n";
            Log.v("BOK", "Intake arm pos " + inPos);
            opMode.idle();
        }
        //File file = AppUtil.getInstance().getSettingsFile("BoKMotorData.csv");
        //ReadWriteFile.writeFile(file,
        //       logString);
        intakeArmMotor.setPower(0);
        opMode.telemetry.addData("BOK ", "Ending PID");
        opMode.telemetry.update();
        isRunningIntakeArmPID = false;
    }

}
