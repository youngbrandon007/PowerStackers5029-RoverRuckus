package org.firstinspires.ftc.teamcode.PSRobotLibs.lib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Brandon on 3/6/2018.
 */

abstract public class PSConfigOpMode extends OpMode {
    public PSRobot robot;

    abstract public void config(OpMode opMode);
}
