package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class BoKAutoRedNear extends BoKAutoCommon {
    // Constructor
    public BoKAutoRedNear()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware()
    {
        BoKAutoCubeLocation cubeLocation = findCube();
        double distMove = 34; // TODO: How much to move when cube is in the middle
        moveRamp(0.5, 33, true, 5);

        if (cubeLocation == BoKAutoCubeLocation.BOK_CUBE_RIGHT) {
            robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_INIT);
        }
        else if (cubeLocation == BoKAutoCubeLocation.BOK_CUBE_LEFT) {
            robot.samplerRightServo.setPosition(robot.SAMPLER_RIGHT_SERVO_INIT);
        }
        else {
            // TODO: move the intake arm
            moveIntake(0.3, 460);
        }

        moveRamp(0.5, 5, true, 5);
        moveIntake(-0.6, 0);
    }
}
