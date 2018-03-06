package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    // Claw wrist
    protected static final double CW_INIT = 0.5;
    protected static final double CW_MIN = 0.1;
    protected static final double CW_MID = 0.4;
    // Claw grab
    protected static final double CG_INIT = 0.6; // Partially open at initialization
    protected static final double CG_OPEN = 1;
    protected static final double CG_CLOSE = 0;
    // Jewel flicker arm
    protected static final double JA_INIT = 0.2;
    protected static final double JA_MID = 0.65;
    protected static final double JA_FINAL = 0.72;
    // Jewel flicker
    protected static final double JF_INIT = 0.9;
    protected static final double JF_FINAL = 0.47;
    protected static final double JF_RIGHT = 1;
    protected static final double JF_LEFT = 0;
    // Glyph flipper
    protected static final double GF_INIT = 0.34;
    protected static final double GF_MID = 0.5;
    protected static final double GF_FINAL_AUTO = 0.6;
    protected static final double GF_FINAL_TELE = 0.7;
    // Glyph flipper gate
    protected static final double FG_UP = 1;
    protected static final double FG_DOWN = 0.22;
    // Relic lift arm
    protected static final double RA_INIT = 0.13;
    protected static final double RA_UPPER_LIMIT = 0.32;
    protected static final double RA_LOWER_LIMIT = 0.64;
    protected static final double RA_HIGH_POS = 0.52;
    protected static final double RA_DEPLOY_POS = 0.59;
    // Relic claw
    protected static final double RC_UNLOCK = 1.0; // initially unlocked
    protected static final double RC_LOCK = 0.2;
    // Glyph arm positions to place the glyph
    protected static final int UA_GLYPH_AT_TIP = 138;
    protected static final int UA_GLYPH_AT_MID = 160;
    protected static final int UA_GLYPH_AT_END = 170;
    // Wrist positions to place the glyph
    //protected static final double CW_GLYPH_AT_TIP = 0.8;
    protected static final double CW_GLYPH_AT_MID = 0.65;
    protected static final double CW_GLYPH_AT_END = 0.65;
    // Glyph arm placement power
    protected static final double UA_MOVE_POWER_UP = 0.8;
    protected static final double UA_MOVE_POWER_DN = 0.8;

    private static final String TURN_TABLE_MOTOR = "tt";
    private static final String UPPER_ARM_MOTOR  = "ua";
    private static final String RELIC_SPOOL_MOTOR = "sp";
    private static final String CLAW_WRIST_SERVO = "cw";
    private static final String CLAW_GRAB_SERVO  = "cg";
    private static final String JEWEL_ARM_SERVO  = "ja";
    private static final String JEWEL_FLICKER_SERVO  = "jf";
    private static final String RELIC_ARM_SERVO = "ra";
    private static final String RELIC_CLAW_SERVO = "rc";
    private static final String GLYPH_FLICKER = "gf";
    private static final String FLIPPER_GATE = "fg";
    private static final String RANGE_SENSOR_JA = "rs";
    private static final String RANGE_SENSOR_FRONT_CFG  = "rsf";
    private static final String RANGE_SENSOR_BACK_CFG   = "rsb";
    private static final String RANGE_SENSOR_GA_CFG = "rsg";
    private static final String COLOR_SENSOR_FRONT_CFG = "csf";
    private static final String COLOR_SENSOR_BACK_CFG = "csb";
    private static final String IMU_TOP = "imu_top";

    protected static final int WAIT_PERIOD = 40; // 40 ms
    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    private static final double CS_SCALE_FACTOR = 255;

    LinearOpMode opMode; // current opMode

    // DC motors
    protected DcMotor turnTable;
    protected DcMotor upperArm;
    protected DcMotor relicSpool;


    // Servos
    protected Servo glyphClawWrist; // These are used in the GlyphArm
    protected Servo glyphClawGrab;
    protected Servo jewelArm;
    protected Servo jewelFlicker;
    protected Servo relicArm;
    protected Servo relicClaw;
    protected Servo glyphFlipper;
    protected Servo flipperGate;

    // Sensors
    protected BNO055IMU imu;

    protected ModernRoboticsI2cRangeSensor rangeSensorJA;
    protected ModernRoboticsI2cRangeSensor rangeSensorFront;
    protected ModernRoboticsI2cRangeSensor rangeSensorBack;
    protected ModernRoboticsI2cRangeSensor rangeSensorGA;

    protected ColorSensor sensorColorFront;
    protected ColorSensor sensorColorBack;

    // Glyph Arm
    BoKGlyphArm glyphArm;
    protected double wristInitPosFromFile = CW_INIT;
    private Orientation angles;

    // waitForTicks
    private ElapsedTime period  = new ElapsedTime();

    // return status
    protected enum BoKHardwareStatus
    {
        BOK_HARDWARE_FAILURE,
        BOK_HARDWARE_SUCCESS
    }

    // hsvValues is an array that will hold the hue, saturation, and value information.
    private float hsvValues[] = {0F, 0F, 0F};

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
        glyphClawWrist = opMode.hardwareMap.servo.get(CLAW_WRIST_SERVO);
        if(glyphClawWrist == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        glyphClawGrab = opMode.hardwareMap.servo.get(CLAW_GRAB_SERVO);
        if(glyphClawGrab == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        jewelArm = opMode.hardwareMap.servo.get(JEWEL_ARM_SERVO);
        if(jewelArm == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        jewelFlicker = opMode.hardwareMap.servo.get(JEWEL_FLICKER_SERVO);
        if(jewelFlicker == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        relicArm = opMode.hardwareMap.servo.get(RELIC_ARM_SERVO);
        if(relicArm == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        relicSpool = opMode.hardwareMap.dcMotor.get(RELIC_SPOOL_MOTOR);
        if(relicSpool == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        relicClaw = opMode.hardwareMap.servo.get(RELIC_CLAW_SERVO);
        if(relicClaw == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        glyphFlipper = opMode.hardwareMap.servo.get(GLYPH_FLICKER);
        if(glyphFlipper == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        flipperGate = opMode.hardwareMap.servo.get(FLIPPER_GATE);
        if(flipperGate == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        upperArm = opMode.hardwareMap.dcMotor.get(UPPER_ARM_MOTOR);
        if(upperArm == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        turnTable = opMode.hardwareMap.dcMotor.get(TURN_TABLE_MOTOR);
        if(turnTable == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        imu = opMode.hardwareMap.get(BNO055IMU.class, IMU_TOP);
        if(imu == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rangeSensorJA = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,
                RANGE_SENSOR_JA);
        if (rangeSensorJA == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rangeSensorFront = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,
                RANGE_SENSOR_FRONT_CFG);
        if (rangeSensorFront == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rangeSensorBack = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,
            RANGE_SENSOR_BACK_CFG);
        if (rangeSensorBack == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        sensorColorFront = opMode.hardwareMap.get(ColorSensor.class, COLOR_SENSOR_FRONT_CFG);
        if (sensorColorFront == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        sensorColorBack = opMode.hardwareMap.get(ColorSensor.class, COLOR_SENSOR_BACK_CFG);
        if (sensorColorBack == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rangeSensorGA = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGE_SENSOR_GA_CFG);
        if (rangeSensorGA == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        if (!opMode.getClass().getName().contains("Tele")) {
            //Make sure that the jewel flicker is open
            jewelFlicker.setPosition(JF_FINAL);


            File file = AppUtil.getInstance().getSettingsFile("BoKArmCalibration.txt");
            String value = ReadWriteFile.readFile(file);
            if (value.isEmpty()) {
                Log.v("BOK", "File not found");
                glyphClawWrist.setPosition(CW_INIT);
            } else {
                Log.v("BOK", "Calibration data: " + value);
                wristInitPosFromFile = Double.parseDouble(value);
                glyphClawWrist.setPosition(wristInitPosFromFile);
            }
        }

        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnTable.setPower(0);

        if (!opMode.getClass().getName().contains("Tele")) {
            glyphClawGrab.setPosition(CG_INIT);
            jewelArm.setPosition(JA_INIT);
            jewelFlicker.setPosition(JF_INIT);
            relicArm.setPosition(RA_INIT);
            relicClaw.setPosition(RC_LOCK);
            glyphFlipper.setPosition(GF_INIT);
            flipperGate.setPosition(FG_DOWN);
        }
        else {
            // IMPORTANT: Do not move the servos during initialization of Teleop
            relicArm.setPosition(RA_INIT);
            flipperGate.setPosition(FG_DOWN);
        }

        //upperArm.setDirection(DcMotorSimple.Direction.REVERSE);
        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //upperArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperArm.setPower(0);

        relicSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicSpool.setPower(0);

        glyphArm = new BoKGlyphArm(this, opMode, glyphClawWrist, glyphClawGrab);
       
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
    protected abstract boolean haveDTMotorsReachedTarget();

    //public abstract void setPowerToDTMotors(double leftPower, double rightPower);
    protected abstract void setPowerToDTMotors(double leftFrontPower,
                                               double leftBackPower,
                                               double rightFrontPower,
                                               double rightBackPower);
    protected abstract void setModeForDTMotors(DcMotor.RunMode runMode);

    // Autonomous driving
    protected abstract int startMove(double leftPower,
                                     double rightPower,
                                     double inches,
                                     boolean backward);

    protected abstract int startStrafe(double power, double rotations,
                                       boolean right);

    protected abstract void stopMove();

    protected abstract double getTargetEncCount(double targetDistanceInches);
    protected abstract int getLFEncCount();
    protected abstract int getRFEncCount();

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

    protected float[] getHue(ColorSensor senseColor)
    {
        Color.RGBToHSV((int) (senseColor.red() * CS_SCALE_FACTOR),
                (int) (senseColor.green() * CS_SCALE_FACTOR),
                (int) (senseColor.blue() * CS_SCALE_FACTOR),
                hsvValues);
        // Log.v("BOK", "Hue: " + hsvValues[0] + ", sat: " + hsvValues[1] + ", val: " + hsvValues[2]);
        return hsvValues;
    }
}
