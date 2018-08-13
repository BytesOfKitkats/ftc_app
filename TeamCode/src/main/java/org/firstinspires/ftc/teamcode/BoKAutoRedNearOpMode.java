package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses BoKMecanumDT and BoKAutoRedNear objects
 */
@Autonomous(name="BoK Auto RED Near", group="BoKRed")
//@Disabled
public class BoKAutoRedNearOpMode extends BoKAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new BoKAutoRedNear(); // use interface (polymorphism)
        super.runOpMode();
    }
}