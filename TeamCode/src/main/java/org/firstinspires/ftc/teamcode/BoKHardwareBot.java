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

    // DC Motors
    private static final String INTAKE_ARM_MOTOR_NAME   = "inA";
    private static final String INTAKE_SLIDE_MOTOR_NAME = "inS";
    private static final String DUMPER_SLIDE_MOTOR_NAME = "duS";
    private static final String HANG_MOTOR_NAME         = "ha";

    // Servos
    private static final String INTAKE_SERVO_NAME        = "inCRS";
    private static final String SAMPLER_LEFT_SERVO_NAME  = "saL";
    private static final String SAMPLER_RIGHT_SERVO_NAME = "saR";
    private static final String DUMPER_ROTATE_SERVO_NAME = "duR";
    private static final String DUMPER_TILT_SERVO_NAME   = "duT";
    private static final String HANG_HOOK_SERVO_NAME     = "haH";

    // Servo positions
    protected static final double SAMPLER_LEFT_SERVO_INIT   = 0.85;
    protected static final double SAMPLER_LEFT_SERVO_FINAL  = 0.2;
    protected static final double SAMPLER_RIGHT_SERVO_INIT  = 0.15;
    protected static final double SAMPLER_RIGHT_SERVO_FINAL = 0.8;
    protected static final double DUMPER_ROTATE_SERVO_INIT  = .77;
    protected static final double DUMPER_ROTATE_SERVO_FINAL = 0.4;
    protected static final double DUMPER_TILT_SERVO_INIT    = 1.0;
    protected static final double DUMPER_TILT_SERVO_FINAL   = 0.0;
    protected static final double HANG_HOOK_SERVO_INIT      = 0.15;
    protected static final double HANG_HOOK_SERVO_FINAL     = 1.0;

    // Sensors
    private static final String IMU_TOP = "imu";        // IMU

    protected static final int WAIT_PERIOD = 40; // 40 ms

    LinearOpMode opMode; // current opMode

    // DC motors
    protected DcMotor intakeArmMotor;
    protected DcMotor intakeSlidesMotor;
    protected DcMotor dumperSlideMotor;
    protected DcMotor hangMotor;

    // Servos
    protected CRServo intakeSweeperServo;
    protected Servo   samplerLeftServo;
    protected Servo   samplerRightServo;
    protected Servo   dumperRotateServo;
    protected Servo   dumperTiltServo;
    protected Servo   hangHookServo;

    // Sensors
    protected BNO055IMU imu;

    private Orientation angles;

    // waitForTicks
    private ElapsedTime period  = new ElapsedTime();

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

        intakeSlidesMotor = opMode.hardwareMap.dcMotor.get(INTAKE_SLIDE_MOTOR_NAME);
        if (intakeSlidesMotor == null) {
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
        intakeSweeperServo = opMode.hardwareMap.crservo.get(INTAKE_SERVO_NAME);
        if (intakeSweeperServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        samplerLeftServo = opMode.hardwareMap.servo.get(SAMPLER_LEFT_SERVO_NAME);
        if (samplerLeftServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        samplerRightServo = opMode.hardwareMap.servo.get(SAMPLER_RIGHT_SERVO_NAME);
        if (samplerRightServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        dumperRotateServo = opMode.hardwareMap.servo.get(DUMPER_ROTATE_SERVO_NAME);
        if (dumperRotateServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        dumperTiltServo = opMode.hardwareMap.servo.get(DUMPER_TILT_SERVO_NAME);
        if (dumperTiltServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        hangHookServo = opMode.hardwareMap.servo.get(HANG_HOOK_SERVO_NAME);
        if (hangHookServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        // Sensors
        imu = opMode.hardwareMap.get(BNO055IMU.class, IMU_TOP);
        if(imu == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        // DC Motor initialization
        intakeArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeSlidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dumperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dumperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dumperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servos initialization
        if (!opMode.getClass().getName().contains("Tele")) {
            samplerLeftServo.setPosition(SAMPLER_LEFT_SERVO_INIT);
            samplerRightServo.setPosition(SAMPLER_RIGHT_SERVO_INIT);
            hangHookServo.setPosition(HANG_HOOK_SERVO_INIT);
        }
        else {
            // Do nothing for Teleop so that the robot hardware does not move during
            // initialization
        }
        dumperRotateServo.setPosition(DUMPER_ROTATE_SERVO_INIT);
        dumperTiltServo.setPosition(DUMPER_TILT_SERVO_INIT);

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
}
