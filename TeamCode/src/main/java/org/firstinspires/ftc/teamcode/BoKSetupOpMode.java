package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import android.util.Log;

/**
 * Created by Krishna Saxena on 11/1/2017.
 */
@TeleOp(name="BOK SETUP", group= "BoKZ")
//@Disabled
public class BoKSetupOpMode extends LinearOpMode
{
    BoKHardwareBot robot = new BoKMecanumDT();

    public void runOpMode()
    {
        boolean test_sensors = false, test_relic_arm = false;
        robot.initHardware(this);

        telemetry.addData("Status", "Hardware initialized");
        telemetry.update();

        waitForStart();
        
        while(opModeIsActive()) {

        }
    }
}
