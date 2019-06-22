package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    protected static final double SPEED_COEFF_MED = 0.5;
    protected static final double SPEED_COEFF_FAST = 0.8;
    protected static final double SPEED_COEFF_TURN = 0.3;
    protected static final double SPEED_COEFF_TURN_FAST = 0.4;
    protected static final double GAME_STICK_DEAD_ZONE = 0.1;

    // DC Motors
    private static final String INTAKE_SLIDE_MOTOR_NAME   = "inS";
    private static final String INTAKE_MOTOR_NAME       = "in";
    private static final String DUMPER_SLIDE_MOTOR_NAME = "duS";
    private static final String HANG_MOTOR_NAME         = "ha";

    // Servos
    private static final String DUMPER_GATE_SERVO_NAME = "duG";
    private static final String INTAKE_GATE_SERVO_NAME   = "inG";
    private static final String INTAKE_LEFT_SERVO_NAME   = "inL";
    private static final String INTAKE_RIGHT_SERVO_NAME   = "inR";
    private static final String MARKER_SERVO_NAME        = "ma";
    private static final String DISTANCE_ROTATE_SERVO_NAME = "dsR";

    // Servo positions
    protected static final double DUMPER_GATE_SERVO_INIT  = 0.3;
    protected static final double DUMPER_GATE_SERVO_FINAL = 0.95;
    protected static final double INTAKE_GATE_SERVO_CLOSED  = 0.82;
    protected static final double INTAKE_GATE_SERVO_OPEN = 0.4;
    protected static final double INTAKE_LEFT_SERVO_DOWN  = 0.95;
    protected static final double INTAKE_LEFT_SERVO_MID  = 0.754;
    protected static final double INTAKE_LEFT_SERVO_UP = 0.503;
    protected static final double INTAKE_LEFT_SERVO_INIT = 0.167;
    protected static final double INTAKE_RIGHT_SERVO_DOWN  = 0.05;
    protected static final double INTAKE_RIGHT_SERVO_MID  = 0.246;
    protected static final double INTAKE_RIGHT_SERVO_UP = 0.497;
    protected static final double INTAKE_RIGHT_SERVO_INIT = 0.833;
    protected static final double MARKER_SERVO_INIT         = 0.31;
    protected static final double MARKER_SERVO_FINAL        = 0.85;
    protected static final double DISTANCE_ROTATE_SERVO_INIT  = 0.52;
    protected static final double DISTANCE_ROTATE_SERVO_FINAL = 0.68;

    // Encoder positions
    protected static final int DUMPER_SLIDE_FINAL_POS    = 1250;
    protected static final int HANG_LIFT_HIGH_POS        = 6790;//6560; //537.6 ppr*0.75 gear*0.315"(8mm) = 1280 pulses/"

    // Sensors
    private static final String IMU_TOP = "imu";        // IMU
    private static final String DISTANCE_SENSOR_BACK = "dsB";
    private static final String DISTANCE_SENSOR_FRONT = "dsF";

    protected static final int WAIT_PERIOD = 40; // 40 ms

    LinearOpMode opMode; // current opMode

    // DC motors
    protected DcMotor intakeSlideMotor;
    protected DcMotor intakeMotor;
    protected DcMotorEx dumperSlideMotor;
    protected DcMotor hangMotor;

    // Servos
    protected Servo   dumperGateServo;
    protected Servo   intakeGateServo;
    protected Servo   intakeLeftServo;
    protected Servo   intakeRightServo;
    protected Servo   markerServo;
    protected Servo   distanceRotateServo;

    // Sensors
    protected BNO055IMU imu;
    protected AnalogInput distanceBack;
    protected AnalogInput distanceFront;

    private Orientation angles;

    // waitForTicks
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime runTime  = new ElapsedTime();

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
        intakeSlideMotor = opMode.hardwareMap.dcMotor.get(INTAKE_SLIDE_MOTOR_NAME);
        if (intakeSlideMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        intakeMotor = opMode.hardwareMap.dcMotor.get(INTAKE_MOTOR_NAME);
        if (intakeMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        dumperSlideMotor = opMode.hardwareMap.get(DcMotorEx.class,DUMPER_SLIDE_MOTOR_NAME);
        if (dumperSlideMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        hangMotor = opMode.hardwareMap.dcMotor.get(HANG_MOTOR_NAME);
        if (hangMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        // Servos
        dumperGateServo = opMode.hardwareMap.servo.get(DUMPER_GATE_SERVO_NAME);
        if (dumperGateServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        markerServo = opMode.hardwareMap.servo.get(MARKER_SERVO_NAME);
        if (markerServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        distanceRotateServo = opMode.hardwareMap.servo.get(DISTANCE_ROTATE_SERVO_NAME);
        if (distanceRotateServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        intakeGateServo = opMode.hardwareMap.servo.get(INTAKE_GATE_SERVO_NAME);
        if (intakeGateServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        intakeLeftServo = opMode.hardwareMap.servo.get(INTAKE_LEFT_SERVO_NAME);
        if (intakeLeftServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        intakeRightServo = opMode.hardwareMap.servo.get(INTAKE_RIGHT_SERVO_NAME);
        if (intakeRightServo == null) {
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

        distanceFront = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_FRONT);
        if(distanceFront == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }


        // DC Motor initialization
        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlideMotor.setPower(0);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setPower(0);

        dumperSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        dumperSlideMotor.setTargetPositionTolerance(25);
        dumperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dumperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dumperSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangMotor.setPower(0);

        // Servos initialization
        if (!opMode.getClass().getName().contains("Tele")) {
            markerServo.setPosition(MARKER_SERVO_INIT);
            dumperGateServo.setPosition(DUMPER_GATE_SERVO_INIT);
            distanceRotateServo.setPosition(DISTANCE_ROTATE_SERVO_FINAL);
            intakeLeftServo.setPosition(INTAKE_LEFT_SERVO_INIT);
            intakeRightServo.setPosition(INTAKE_RIGHT_SERVO_INIT);
            intakeGateServo.setPosition(INTAKE_GATE_SERVO_CLOSED);
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
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //angles = new Orientation();
        imu.initialize(parameters);
    }

    // Initialization of drive train is protected but abstract
    protected abstract BoKHardwareStatus initDriveTrainMotors();

    // Using the drive train is public
    protected abstract void testDTMotors();
    protected abstract void resetDTEncoders();
    protected abstract boolean areDTMotorsBusy();

    protected abstract void setPowerToDTMotors(double power);
    protected abstract void setPowerToDTMotors(double power, boolean forward);
    protected abstract void setPowerToDTMotors(double leftPower, double rightPower);
    protected abstract void setPowerToDTMotorsStrafe(double power, boolean right);
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


    protected abstract int startStrafe(double power, double rotations,
                                       boolean right) throws UnsupportedOperationException;

    protected abstract int startStrafeWEnc(double power, double rotations,
                                       boolean right) throws UnsupportedOperationException;

    protected abstract void stopMove();

    protected abstract double getTargetEncCount(double targetDistanceInches);
    protected abstract double getAvgEncCount();
    protected abstract int getLFEncCount();
    protected abstract int getRFEncCount();

    // Teleop driving
    protected abstract void moveRobotTele(double speedCoef, double turnCoef, boolean endGame);

    /*
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     *b The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
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
}
