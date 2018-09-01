
package org.firstinspires.ftc.teamcode.PSRobotLibs.examples.configExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Brandon on 3/31/2017.
 */

@TeleOp(name = "PineEx-Config", group = "Linear Opmode")
@Disabled


public class Code extends Config {

    @Override
    public void runOpMode() throws InterruptedException {

        config(this);

        waitForStart();

        while (opModeIsActive()) {

            testMotor.update(gamepad1.left_stick_x);
        }
    }

}

