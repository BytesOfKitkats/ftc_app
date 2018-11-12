package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueFar extends BoKAutoCommon
{
    // Constructor
    public BoKAutoBlueFar()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
        far = true;
    }

    @Override
    public void runSoftware()
    {
        //gyroTurn(0.5, 0, 75, DT_TURN_THRESHOLD_LOW,
        //        false, false, 5);
        //followHeadingPID(75, 0.5, 24, 6);
        //moveIntake(0.3, 30);
        //moveWithRangeSensor(0.5, 225, 160,true, 7);
        moveIntake(0.5, 780);
        sweepRoller(-1);
        opMode.sleep(1000);
        sweepRoller(0);
        moveIntake(0.5, 10);
    }
}
