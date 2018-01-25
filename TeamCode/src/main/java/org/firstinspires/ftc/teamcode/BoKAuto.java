package org.firstinspires.ftc.teamcode;

/**
 * Created by Krishna Saxena on 10/14/2016.
 * Define all the CONSTANTS used for autonomous mode.
 */
public interface BoKAuto
{
    public enum BoKAllianceColor {
        BOK_ALLIANCE_RED,
        BOK_ALLIANCE_BLUE
    }

    public enum BoKAutoStatus {
        BOK_AUTO_FAILURE,
        BOK_AUTO_SUCCESS
    }

    public static long WAIT_FOR_SERVO_MS = 350;
    public static double VUFORIA_TIMEOUT = 4.0;
    //public static double CRS_CRYPTO_TIMEOUT = 2.5;
    //public static double DT_STRAFE_TIMEOUT = 3.0;
    public static double DT_TURN_TIMEOUT = 5.0;
    public static double BLUE_CRYPTO_MOVE_TIMEOUT = 5.0;

    public static double DT_POWER_FOR_STONE = 0.25;
    public static double DT_POWER_FOR_LINE = 0.2;
    //public static double DT_POWER_FOR_STRAFE = 0.2;
    public static double DT_POWER_FOR_RS = 0.15;
    public static double DT_POWER_FOR_CRYPTO = 0.12;
    public static double DT_TURN_SPEED_LOW  = 0.15;
    public static double DT_TURN_SPEED_HIGH = 0.4;
    //public static double ROTATIONS_STRAFE_TO_WALL = 0.15;
    public static double DT_RAMP_SPEED_INIT = 0.15;
    public static int TURN_LEFT_DEGREES = 90;
    public static double UA_INIT_ANGLE = 12;
    public static double DISTANCE_OFF_BALANCE = 20; // inches
    public static double DISTANCE_AWAY_FROM_CRYPTO = 3.25;//3.25
    public static double DT_TIMEOUT = 4;
    public static double UA_TIMEOUT = 2;
    public static double GF_TIMEOUT = 2;

    public BoKAutoStatus initSoftware(BoKAutoOpMode opMode,
                                      BoKHardwareBot robot);

    public void runSoftware();
}
