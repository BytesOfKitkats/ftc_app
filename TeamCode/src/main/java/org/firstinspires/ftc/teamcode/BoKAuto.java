package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

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

    // Jewel flicker wait
    public static int WAIT_FOR_JEWEL_FLICKER_MS = 250;
    // Vuforia
    public static int VUFORIA_TIMEOUT = 4;
    // Drive train
    public static double DT_POWER_FOR_STONE = 0.25;
    public static double DT_POWER_HIGH = 0.4;
    public static double DT_RAMP_SPEED_INIT = 0.15;
    public static double DT_POWER_FOR_FLICK = 0.2;
    public static double DT_POWER_FOR_CRYPTO = 0.15;
    public static int DT_TIMEOUT_2S = 2;
    public static int DT_TIMEOUT_4S = 4;
    public static int DT_TIMEOUT_5S = 5;
    public static int DT_TIMEOUT_6S = 6;
    //public static double DT_STRAFE_TIMEOUT = 3.0;
    // Drive train turn
    public static int DT_TURN_THRESHOLD_LOW = 1;
    public static int DT_TURN_THRESHOLD_HIGH = 2;
    public static int DT_TURN_TIMEOUT = 5;
    public static double DT_TURN_SPEED_LOW  = 0.25;
    public static double DT_TURN_SPEED_HIGH = 0.4;
    public static int TURN_LEFT_DEGREES = 90;
    public static double DT_POWER_FOR_STRAFE = 0.2;
    //public static double ROTATIONS_STRAFE_TO_WALL = 0.15;

    // flipFlipper constants
    public static int FLIP_FLIPPER_INIT = 1;
    public static int FLIP_FLIPPER_DUMP = 2;
    public static int FLIP_FLIPPER_LOWER = 3;
    public static int RAISE_LIFT = 4;
    public static int LOWER_LIFT_AND_RESET_FLIPPER = 5;

    // Rollers constants
    public static int INTAKE_ROLLERS = 1;
    public static int REVERSE_ROLLERS = 2;

    public static ElapsedTime runTimeOpMode = new ElapsedTime();
    public static ElapsedTime runTime2ndGlyph = new ElapsedTime();
    public BoKAutoStatus initSoftware(BoKAutoOpMode opMode,
                                      BoKHardwareBot robot);

    public void runSoftware();
}
