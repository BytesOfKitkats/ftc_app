package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BoK Auto Test", group="BoKTest")
public class BoKAutoTestOpMode extends BoKAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new BoKAutoTest(); // use interface (polymorphism)
        super.runOpMode();
    }
}
