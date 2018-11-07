package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoBlueNear extends BoKAutoCommon
{
    // Constructor
    public BoKAutoBlueNear()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        BoKAutoCubeLocation cubeLocation = findCube();
        double distMove = 13; // TODO: How much to move when cube is in the middle
        moveRamp(0.5, distMove, true, 5);

        if (cubeLocation == BoKAutoCubeLocation.BOK_CUBE_RIGHT) {

            moveRamp(0.5, 24, true, 5);
            robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_INIT);
        }
        else if (cubeLocation == BoKAutoCubeLocation.BOK_CUBE_LEFT) {

            moveRamp(0.5, 24, true, 5);
            robot.samplerLeftServo.setPosition(robot.SAMPLER_LEFT_SERVO_INIT);
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
