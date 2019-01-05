package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses BoKMecanumDT and BoKAutoRedDepot objects
 */
@Autonomous(name="BoK Auto Depot", group="BoKRed")
//@Disabled
public class BoKAutoRedDepotOpMode extends BoKAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new BoKAutoRedDepot(); // use interface (polymorphism)
        super.runOpMode();
    }
}