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

    // Sensors
    private static final String MAX_BOTIX_BACK_CFG = "mbb"; // Analog
    private static final String MAX_BOTIX_SIDE_RIGHT_CFG = "mbsr";
    private static final String MAX_BOTIX_SIDE_LEFT_CFG = "mbsl";
    private static final String MAX_BOTIX_FRONT_CFG = "mbf";
    private static final String COLOR_SENSOR_BOTTOM = "crb";
    private static final String IMU_TOP = "imu_top";        // IMU

    protected static final int WAIT_PERIOD = 40; // 40 ms

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    private static final double CS_SCALE_FACTOR = 255;
    LinearOpMode opMode; // current opMode

    //Motors

    // Servos

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

    // hsvValues is an array that will hold the hue, saturation, and value information.
    private float hsvValues[] = {0F, 0F, 0F};

    /*
     * The initHardware() method initializes the hardware on the robot including the drive train.
     * It calls the abstract  n() and initMotorsAndSensors() methods.
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
        imu = opMode.hardwareMap.get(BNO055IMU.class, IMU_TOP);
        if(imu == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        if (!opMode.getClass().getName().contains("Tele")) {
            // Servos
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

    protected double getDistanceCM(AnalogInput mb1240, double maxDist)
    {
        double dist =  mb1240.getVoltage() / 0.00189;
        if (dist > maxDist)
            dist = maxDist;
        return dist;
    }
}
