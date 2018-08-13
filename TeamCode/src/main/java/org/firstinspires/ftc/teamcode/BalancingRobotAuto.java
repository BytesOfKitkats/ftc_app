package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by krish on 7/20/2018.
 */
@Autonomous(name = "Balancing RObot", group = "Balancing")
@Disabled
public class BalancingRobotAuto extends LinearOpMode {
    DcMotor left, right;
    BNO055IMU imu;

    int steering = 0;
    int acceleration = 50;
    int speed = 0;

    float gn_dth_dt, gn_th, gn_y, gn_dy_dt, kp, ki, kd, mean_reading, gear_down_ratio, dt,
            your_wheel_diameter = 101.6f;
    float radius, u, th, th_cal, dth_dt, e, de_dt, _edt, e_prev, pid, y, dy_dt, v, y_ref;
    int last_steering, straight;
    float motorpower, d_pwr, factor_mm;
    int n_max = 7, n, n_comp;
    int[] encoder;
    ElapsedTime T4;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.dcMotor.get("lm");
        if (left == null) {
            throw new NullPointerException("No Left Motor");
        }
        right = hardwareMap.dcMotor.get("rm");
        if (left == null) {
            throw new NullPointerException("No Right Motor");
        }
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        if (imu == null) {
            throw new NullPointerException("No IMU");
        }
        // Setup left and right motors
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setPower(0);
        right.setPower(0);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initializeImu();
        setupVariables();

        telemetry.addData("init", "Imu initialized");
        telemetry.update();

        waitForStart();

        T4 = new ElapsedTime();
        T4.reset();
        while (opModeIsActive()) {
            balancing();
            //left.setPower(-0.3);
            //right.setPower(-0.3);
            //sleep(10);
        }
        left.setPower(0);
        right.setPower(0);
    }

    protected void initializeImu() {
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

    void setupVariables(){
        gear_down_ratio = 1;      // no gear down
        dt = 0.030f;              // in seconds
        gn_dth_dt = 0.23f;
        gn_th = 25.00f;
        gn_y = 272.8f;
        gn_dy_dt = 24.6f;
        kp = 0.0336f;
        ki = 0.2688f;
        kd = 0.000504f;
        encoder = new int[n_max];                 // Array containing last n_max motor positions
        radius = your_wheel_diameter / 1000;
        //degtorad = (float) (Math.PI / 180);
        factor_mm = (float)(Math.PI / 144);

        //SETUP VARIABLES FOR CALCULATIONS
        u = 0;                    // Sensor Measurement (raw)
        th = 0;//Theta            // Angle of robot (degree)
        dth_dt = 0;//dTheta/dt    // Angular velocity of robot (degree/sec)
        e = 0;//Error             // Sum of four states to be kept zero: th, dth_dt, y, dy_dt.
        de_dt = 0;//dError/dt     // Change of above error
        _edt = 0;//Integral Error // Accumulated error in time
        e_prev = 0;//Previous Error/ Error found in previous loop cycle
        pid = 0;                  // SUM OF PID CALCULATION
        y = 0;//y                     // Measured Motor position (degrees)
        dy_dt = 0;//dy/dt             // Measured motor velocity (degrees/sec)
        v = 0;//velocity          // Desired motor velocity (degrees/sec)
        y_ref = 0;//reference pos // Desired motor position (degrees)
        motorpower = 0;             // Power ultimately applied to motors
        last_steering = 0;          // Steering value in previous cycle
        straight = 0;               // Average motor position for synchronizing
        d_pwr = 0;                  // Change in power required for synchronizing
        n_max = 7;            // Number of measurement used for floating motor speed average
        n = 0; n_comp = 0;           // Intermediate variables needed to compute measured motor speed
        mean_reading = 0;
        th_cal = 90;
    }

    void balancing() {
        //if (true) {

            // calculate error in -179 to +180 range  (
            //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
            //        AxesOrder.XYZ,
            //        AngleUnit.DEGREES);

            //float y_rot = imu.getAngularVelocity().yRotationRate;
            float u = imu.getAngularVelocity().zRotationRate;
            //float x_rot = imu.getAngularVelocity().xRotationRate;
            //double currT = T4.milliseconds();

            //COMPUTE GYRO ANGULAR VELOCITY AND ESTIMATE ANGLE
            dth_dt = u / 2 - mean_reading;
            mean_reading = (float) (mean_reading * 0.999 + (0.001 * (dth_dt + mean_reading)));
            th_cal = th_cal + dth_dt*(float)dt;
            //th = angles.secondAngle;
            //float x_ang = angles.firstAngle;
            //float z_ang = angles.thirdAngle;
            th = imu.getAngularOrientation().thirdAngle;
            //float x_ang = imu.getAngularOrientation().firstAngle;
            //float z_ang = imu.getAngularOrientation().thirdAngle;

            //while ((T4.milliseconds() < dt*1000) && (opModeIsActive())) {
                //sleep(1);
            //}
            //currT = T4.milliseconds();
            //Log.v("bal", "t: " + currT + ", u: " + u + ", dth_dt: " + dth_dt + ", th: " + th + ", cal th: " + th_cal);
            //T4.reset();
            //return;
        //}
        //ADJUST REFERENCE POSITION ON SPEED AND ACCELERATION
        if (v < speed * 10) {
            v = v + acceleration * 10 * dt;
        } else if (v > speed * 10) {
            v = v - acceleration * 10 * dt;
        }
        y_ref = y_ref + v * dt;

        //COMPUTE MOTOR ENCODER POSITION AND SPEED
        n++;
        if (n == n_max) {
            n = 0;
        }
        encoder[n] = (int)(left.getCurrentPosition() + right.getCurrentPosition() + y_ref);
        n_comp = n + 1;
        if (n_comp == n_max) {
            n_comp = 0;
        }
        y = encoder[n] * factor_mm * radius / gear_down_ratio;
        dy_dt = (encoder[n] - encoder[n_comp]) / (dt * (n_max - 1)) * factor_mm * radius / gear_down_ratio;

        //COMPUTE COMBINED ERROR AND PID VALUES
        Log.v("bal", "th: " + th + ", cal th: " + th_cal + ", dth_dt: " + dth_dt + ", y: " + y + ", dy_dt :" + dy_dt);
        e = gn_th * (th_cal - 90) + gn_dth_dt * dth_dt + gn_y * y + gn_dy_dt * dy_dt;
        de_dt = (e - e_prev) / dt;
        _edt = _edt + e * dt;
        e_prev = e;
        pid = (kp * e + ki * _edt + kd * de_dt) / radius * gear_down_ratio;

        //ADJUST MOTOR SPEED TO STEERING AND SYNCHING
        if (steering == 0) {
            if (last_steering != 0) {
                straight = right.getCurrentPosition() - left.getCurrentPosition();
            }
            d_pwr = ((right.getCurrentPosition() - left.getCurrentPosition() - straight) / (radius * 10 / gear_down_ratio));
        } else {
            d_pwr = steering / (radius * 10 / gear_down_ratio);
        }
        last_steering = steering;

        //CONTROL MOTOR POWER AND STEERING
        motorpower = pid/100;
        if((motorpower<0.1)&&(motorpower>-0.1)){
            motorpower = 0;
        }
        else if (motorpower > 1){
            motorpower = 1f;
        }
        else if (motorpower <-1){
            motorpower = -1f;
        }
        left.setPower(motorpower + d_pwr);
        right.setPower(motorpower - d_pwr);

        //ERROR CHECKING OR SHUTDOWN
        //if (Math.abs(th) > 60 || Math.abs(motorpower) > 2000) {
        //    stop();
        //}
        Log.v("bal", "e: " + e + ", de_dt: " + de_dt + ", _edt: " + _edt + ", pid: " + pid +",motorpower " + motorpower);

        //WAIT THEN REPEAT
        while ((T4.milliseconds() < dt*1000) && (opModeIsActive())) {
            sleep(1);
        }
        T4.reset();
    }
}

