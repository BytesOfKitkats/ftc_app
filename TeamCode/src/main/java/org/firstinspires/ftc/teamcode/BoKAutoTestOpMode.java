package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="BoK Auto Test", group="BOKTest")
//@Disabled
public class BoKAutoTestOpMode extends BoKAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new BoKAutoTest(); // use interface (polymorphism)
        super.runOpMode();
    }
}
