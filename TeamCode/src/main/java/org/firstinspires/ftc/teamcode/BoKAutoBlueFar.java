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
        BoKAutoCubeLocation cubeLocation = findCube();
        double distMove = 15; // TODO: How much to move when cube is in the middle
        moveRamp(0.5, distMove, true, 5);

        if (cubeLocation == BoKAutoCubeLocation.BOK_CUBE_RIGHT) {
            robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_INIT);
            moveRamp(0.5, 18, true, 5);
        }
        else if (cubeLocation == BoKAutoCubeLocation.BOK_CUBE_LEFT) {
            robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_INIT);
            moveRamp(0.5, 18, true, 5);
        }
        else {
            // TODO: move the intake arm
            moveIntake(0.3, 460);
            sweepRoller(1);
            moveRamp(0.5, 8, true, 5);
        }

        moveRamp(0.5, 5, true, 5);
        sweepRoller(0);
        moveIntake(-0.6, 0);
        moveRamp(0.5, 7, true, 5);

    }
}
