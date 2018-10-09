package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses BoKMecanumDT and BoKAutoRedFar objects
 */
@Autonomous(name="BoK Auto RED Far", group="BoKRed")
//@Disabled
public class BoKAutoRedFarOpMode extends BoKAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new BoKAutoRedFar(); // use interface (polymorphism)
        super.runOpMode();
    }
}