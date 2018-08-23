package org.firstinspires.ftc.teamcode.PSRobotPackage.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Brandon on 3/6/2018.
 */

abstract public class PSConfigOpMode extends OpMode {
    public PSRobot robotHandler;

    abstract public void config(OpMode opMode);
}
