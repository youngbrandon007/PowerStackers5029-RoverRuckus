package org.firstinspires.ftc.teamcode.RelicRecoveryFinalRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Brandon on 1/8/2018.
 */
@Autonomous(name = "Switch Board")
@Disabled
public class SwitchBoard extends Config {
    @Override
    public void runOpMode() throws InterruptedException {
        config(this);
        waitForStart();
        while (opModeIsActive()){
            loadSwitchBoard();
            displaySwitchBorad(true);
            telemetry.update();
        }
    }
}
