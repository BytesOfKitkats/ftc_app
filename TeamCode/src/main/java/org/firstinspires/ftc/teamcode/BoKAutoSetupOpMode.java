package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BoK Auto Setup", group="BOKTest")
//@Disabled
public class BoKAutoSetupOpMode extends BoKAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new BoKAutoSetup(); // use interface (polymorphism)
        super.runOpMode();
    }
}
