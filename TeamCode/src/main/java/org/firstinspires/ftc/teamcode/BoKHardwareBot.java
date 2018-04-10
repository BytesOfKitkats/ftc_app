package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    // Jewel flicker arm
    protected static final double JA_INIT = 0.26;
    protected static final double JA_MID = 0.65;
    protected static final double JA_FINAL = 0.8;
    // Jewel flicker
    protected static final double JF_INIT = 1;
    protected static final double JF_FINAL = 0.47;
    protected static final double JF_RIGHT = 1;
    protected static final double JF_LEFT = 0;
    // Glyph flipper
    protected static final double FLIPPER_DOWN_POS = 0.95;
    protected static final double FLIPPER_UP_POS = 0.5;
    protected static final double FLIPPER_ANGLE_POS = 0.8;
    protected static final double FLIPPER_INIT_POS = 0.95;
    // Roller motor power
    protected static final double ROLLER_POWER_HIGH = 0.95;
    protected static final double ROLLER_POWER_MID = 0.65;
    protected static final double ROLLER_POWER_LOW = 0.4;
    // Relic lift arm
    protected static final double RA_INIT = 0.753;
    protected static final double RA_RAISED_POS = 0.638;
    protected static final double RA_NEAR_POS = 0.52;
    protected static final double RA_FAR_POS = 0.54;
    // Relic claw
    protected static final double RC_UNLOCK = 0.5;
    protected static final double RC_LOCK = 0.9;
    // Roller gates right
    protected static final double RGR_UNLOCK = 1;
    protected static final double RGR_LOCK = 0.6;
    // Roller gates left
    protected static final double RGL_UNLOCK = 0.04;
    protected static final double RGL_LOCK = 0.35;

    // DC Motors
    private static final String RELIC_SPOOL_MOTOR = "sp";
    private static final String LEFT_ROLLER_MOTOR = "il";
    private static final String RIGHT_ROLLER_MOTOR = "ir";
    private static final String FLIPPER_LIFT_MOTOR = "fl";

    // Servos
    private static final String JEWEL_ARM_SERVO  = "ja";
    private static final String JEWEL_FLICKER_SERVO  = "jf";
    private static final String RELIC_ARM_SERVO = "rp";
    private static final String RELIC_CLAW_SERVO = "rg";
    private static final String FLIPPER_SERVO = "fls";
    private static final String FLIPPER_RIDING_GATE_LEFT_SERVO = "rgl";
    private static final String FLIPPER_RIDING_GATE_RIGHT_SERVO = "rgr";

    // Sensors
    private static final String RANGE_SENSOR_JA = "rs";     // Modern Robotics
    private static final String MAX_BOTIX_BACK_CFG = "mbb"; // Analog
    private static final String MAX_BOTIX_SIDE_CFG = "mbs";
    private static final String MAX_BOTIX_FRONT_CFG = "mbf";
    private static final String COLOR_SENSOR_NEAR = "crn";  // REV color/proximity
    private static final String COLOR_SENSOR_FAR = "crf";
    private static final String COLOR_SENSOR_BOTTOM = "crb";
    private static final String IMU_TOP = "imu_top";        // IMU

    protected static final int WAIT_PERIOD = 40; // 40 ms
    private static final int RELIC_SPOOL_PLAY_POS = 250;
    private static final double RELIC_SPOOL_PLAY_POWER = 0.2;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    private static final double CS_SCALE_FACTOR = 255;
    LinearOpMode opMode; // current opMode

    // DC motors
    protected DcMotor relicSpool;
    protected DcMotor leftRoller;
    protected DcMotor rightRoller;
    protected DcMotor flipperLift;

    // Servos
    protected Servo jewelArm;
    protected Servo jewelFlicker;
    protected Servo relicArm;
    protected Servo relicClaw;
    protected Servo ridingGateLeft;
    protected Servo ridingGateRight;
    protected Servo flipper;

    // Sensors
    protected BNO055IMU imu;
    protected AnalogInput mb1240Back, mb1240Side, mb1240Front;
    protected ModernRoboticsI2cRangeSensor rangeSensorJA;
    protected DistanceSensor distNear, distFar;
    protected ColorSensor colorNear, colorFar, colorBottom;

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
        // type here
        leftRoller = opMode.hardwareMap.dcMotor.get(LEFT_ROLLER_MOTOR);
        if(leftRoller == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rightRoller = opMode.hardwareMap.dcMotor.get(RIGHT_ROLLER_MOTOR);
        if(rightRoller == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        flipperLift = opMode.hardwareMap.dcMotor.get(FLIPPER_LIFT_MOTOR);
        if(flipperLift == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        relicSpool = opMode.hardwareMap.dcMotor.get(RELIC_SPOOL_MOTOR);
        if(relicSpool == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        flipper = opMode.hardwareMap.servo.get(FLIPPER_SERVO);
        if(flipper == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }


        ridingGateLeft = opMode.hardwareMap.servo.get(FLIPPER_RIDING_GATE_LEFT_SERVO);
        if(ridingGateLeft == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        ridingGateRight = opMode.hardwareMap.servo.get(FLIPPER_RIDING_GATE_RIGHT_SERVO);
        if(ridingGateRight == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        relicArm = opMode.hardwareMap.servo.get(RELIC_ARM_SERVO);
        if(relicArm == null){
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

        relicClaw = opMode.hardwareMap.servo.get(RELIC_CLAW_SERVO);
        if(relicClaw == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        imu = opMode.hardwareMap.get(BNO055IMU.class, IMU_TOP);
        if(imu == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        mb1240Back = opMode.hardwareMap.analogInput.get(MAX_BOTIX_BACK_CFG);
        if(mb1240Back == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        mb1240Front = opMode.hardwareMap.analogInput.get(MAX_BOTIX_FRONT_CFG);
        if(mb1240Front == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        mb1240Side = opMode.hardwareMap.analogInput.get(MAX_BOTIX_SIDE_CFG);
        if(mb1240Side == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        distNear = opMode.hardwareMap.get(DistanceSensor.class, COLOR_SENSOR_NEAR);
        if(distNear == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        colorNear = opMode.hardwareMap.get(ColorSensor.class, COLOR_SENSOR_NEAR);

        distFar = opMode.hardwareMap.get(DistanceSensor.class, COLOR_SENSOR_FAR);
        if(distFar == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        colorFar = opMode.hardwareMap.get(ColorSensor.class, COLOR_SENSOR_FAR);

        colorBottom = opMode.hardwareMap.get(ColorSensor.class, COLOR_SENSOR_BOTTOM);
        if(colorBottom == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        // Rollers DC Motors
        rightRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRoller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Relic spool DC Motor
        relicSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicSpool.setPower(0);
        // Flipper lift DC Motor
        flipperLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipperLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipperLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipperLift.setDirection(DcMotorSimple.Direction.REVERSE);
        flipperLift.setPower(0);

        rangeSensorJA = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,
                RANGE_SENSOR_JA);
        if (rangeSensorJA == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        if (!opMode.getClass().getName().contains("Tele")) {
            // Servos
            flipper.setPosition(FLIPPER_INIT_POS);
            ridingGateLeft.setPosition(RGL_LOCK);
            ridingGateRight.setPosition(RGR_LOCK);
            jewelArm.setPosition(JA_INIT);
            jewelFlicker.setPosition(JF_INIT);
            //relicArm.setPosition(RA_INIT);
            relicClaw.setPosition(RC_LOCK);
            //initRelicArm(); cannot initialize relic arm as it goes over 18"
        }
        else {
           // IMPORTANT: Do not move the servos during initialization of Teleop

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

    protected abstract void startEncMove(double leftPower,
                                         double rightPower,
                                         int encCount,
                                         boolean forward);

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
        //Log.v("BOK", "Hue: " + hsvValues[0] +
        //             ", Sat: " + hsvValues[1] +
        //             ", Val: " + hsvValues[2] +
        //            ", Alpha: " + senseColor.alpha());
        return hsvValues;
    }

    protected void initRelicArm()
    {
        ElapsedTime trackTime = new ElapsedTime();

        // Raise the relic arm
        relicArm.setPosition(RA_INIT); // raise the relic arm
        // Bring the relic out so that it is locked
        relicSpool.setTargetPosition(RELIC_SPOOL_PLAY_POS);
        relicSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicSpool.setPower(RELIC_SPOOL_PLAY_POWER);

        trackTime.reset();
        while (relicSpool.isBusy()) {
            if (trackTime.seconds() >= 2) {
                Log.v("BOK", "Relic spool motor timed out during initRelicArm!");
                break;
            }
        }
        relicSpool.setPower(0);
        relicSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected double getDistanceCM(AnalogInput mb1240)
    {
        return mb1240.getVoltage() / 0.00189;
    }
}
